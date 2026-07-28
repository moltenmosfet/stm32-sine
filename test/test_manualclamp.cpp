/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2026 Molten MOSFET
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* G8 [FORK]: ClampManualCurrent (include/manualclamp.h) is the firmware-side
 * authority cap on the CAN manualiq/manualid torque path, extracted pure so the
 * host harness drives the real clamp. In pwmgeneration-foc.cpp it is applied
 * ONCE per control step:
 *
 *     s32fp manualIqMax = Param::Get(Param::manualiqmax);
 *     s32fp manualIqCmd = ClampManualCurrent(Param::Get(Param::manualiq), manualIqMax);
 *     s32fp manualIdCmd = ClampManualCurrent(Param::Get(Param::manualid), manualIqMax);
 *
 * and the clamped values feed every summing junction (both the ZOE and non-ZOE
 * branches, iq and id). These cases prove: the 400 A default is a pure pass-
 * through (no behaviour change until configured down); the clamp bounds BOTH
 * command polarities of BOTH channels once configured down; and the clamp is
 * defensive against an out-of-range (negative) ceiling. The clamp operates on
 * the raw s32fp representation, so the values here are FP-scaled amps.
 *
 * The ceiling itself is raise-proofed at runtime by EffectiveManualCeiling =
 * MIN(live param, boot-latched value): the host can lower the cap live but a
 * runtime raise only takes hold after set+flash-save+reboot re-latches. Those
 * cases drive the pure guard directly with the latch injected.
 *
 * The `manualiqmax` param is a persisted PARAM_ENTRY (TYPE_PARAM) — it survives
 * a power cycle, which is what makes a commissioned ceiling durable rather than
 * resetting to the 400 default every boot. It is FOC-only, so — like
 * manualiq/iqtimeout — it does not exist in the SINE param branch the host suite
 * links (its own min/max/default are firmware-compile-checked instead).
 * TestParamSetRejects pins the shared range guard on a bounded param that IS in
 * the SINE build, so a regression that stopped Param::Set enforcing ranges (the
 * guard that stops a host writing manualiqmax past 400) fails here. */

#include "manualclamp.h"
#include "params.h"
#include "my_math.h"
#include "test.h"

class ManualClampTest: public UnitTest
{
   public:
      ManualClampTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

static const int32_t MAX400 = FP_FROMINT(400); // default manualiqmax

/* Default ceiling (400 A) is a no-op: any in-range command, either sign,
 * passes through unchanged — the shipped-default binary is behaviourally
 * identical to the pre-G8 ±400-param firmware. */
static void TestDefaultMaxIsNoOp()
{
   ASSERT(ClampManualCurrent(FP_FROMINT(0),    MAX400) == FP_FROMINT(0));
   ASSERT(ClampManualCurrent(FP_FROMINT(150),  MAX400) == FP_FROMINT(150));
   ASSERT(ClampManualCurrent(FP_FROMINT(-150), MAX400) == FP_FROMINT(-150));
   ASSERT(ClampManualCurrent(FP_FROMINT(400),  MAX400) == FP_FROMINT(400));  // boundary
   ASSERT(ClampManualCurrent(FP_FROMINT(-400), MAX400) == FP_FROMINT(-400)); // boundary
}

/* Configured down, the clamp bounds the manualiq magnitude on BOTH polarities;
 * in-window commands still pass through untouched. */
static void TestClampsIqBothSigns()
{
   const int32_t cap = FP_FROMINT(50);

   ASSERT(ClampManualCurrent(FP_FROMINT(150),  cap) == FP_FROMINT(50));  // + over -> +cap
   ASSERT(ClampManualCurrent(FP_FROMINT(-150), cap) == FP_FROMINT(-50)); // - over -> -cap
   ASSERT(ClampManualCurrent(FP_FROMINT(50),   cap) == FP_FROMINT(50));  // at cap, untouched
   ASSERT(ClampManualCurrent(FP_FROMINT(-50),  cap) == FP_FROMINT(-50));
   ASSERT(ClampManualCurrent(FP_FROMINT(30),   cap) == FP_FROMINT(30));  // in-window, untouched
   ASSERT(ClampManualCurrent(FP_FROMINT(-30),  cap) == FP_FROMINT(-30));
}

/* Same clamp gates the manualid (flux) channel — the call site applies it to
 * manualid with the same manualiqmax, so a runaway flux command is bounded too.
 * (Same function; this pins the id channel's coverage explicitly.) */
static void TestClampsIdBothSigns()
{
   const int32_t cap = FP_FROMINT(20);

   ASSERT(ClampManualCurrent(FP_FROMINT(300),  cap) == FP_FROMINT(20));
   ASSERT(ClampManualCurrent(FP_FROMINT(-300), cap) == FP_FROMINT(-20));
   ASSERT(ClampManualCurrent(FP_FROMINT(0),    cap) == FP_FROMINT(0));

   // A zero ceiling fully disables manual authority: everything clamps to 0.
   ASSERT(ClampManualCurrent(FP_FROMINT(400),  0) == 0);
   ASSERT(ClampManualCurrent(FP_FROMINT(-400), 0) == 0);
}

/* A negative ceiling can't come from the range-guarded param (min=0), but the
 * clamp must still be self-defensive: a negative maxMag is treated as 0 (full
 * lockout), never as an inverted window that would pass everything. */
static void TestNegativeCeilingIsDefensive()
{
   ASSERT(ClampManualCurrent(FP_FROMINT(200),  FP_FROMINT(-10)) == 0);
   ASSERT(ClampManualCurrent(FP_FROMINT(-200), FP_FROMINT(-10)) == 0);
   ASSERT(ClampManualCurrent(0,                FP_FROMINT(-10)) == 0);
}

/* Boot latch: a runtime attempt to RAISE the ceiling over CAN has no live
 * effect — the effective ceiling stays pinned at the boot-latched value, so a
 * host bug cannot restore authority it wasn't commissioned with. */
static void TestLatchBlocksRuntimeRaise()
{
   const int32_t latched = FP_FROMINT(50); // commissioned + flash-loaded ceiling

   // Host writes manualiqmax up to 400 at runtime: effective stays 50.
   ASSERT(EffectiveManualCeiling(FP_FROMINT(400), latched) == FP_FROMINT(50));
   // ...and a command is still clamped to the latched ceiling, not the raise.
   ASSERT(ClampManualCurrent(FP_FROMINT(300),
             EffectiveManualCeiling(FP_FROMINT(400), latched)) == FP_FROMINT(50));
}

/* A runtime LOWER takes effect immediately: MIN picks the lower live value, so
 * an operator/host can always de-rate authority without a reboot. */
static void TestLatchAllowsRuntimeLower()
{
   const int32_t latched = FP_FROMINT(50);

   ASSERT(EffectiveManualCeiling(FP_FROMINT(20), latched) == FP_FROMINT(20));
   ASSERT(ClampManualCurrent(FP_FROMINT(300),
             EffectiveManualCeiling(FP_FROMINT(20), latched)) == FP_FROMINT(20));
   // Equal live/latched (fresh board at default, or just-re-latched): pass-through.
   ASSERT(EffectiveManualCeiling(FP_FROMINT(400), FP_FROMINT(400)) == FP_FROMINT(400));
}

/* After set+save+reboot the latch is re-taken at the new (higher) value; from
 * then on the raised ceiling is live. Modelled by latching at the new value. */
static void TestReLatchAppliesRaise()
{
   // Pre-reboot: latched 50 pins the effective ceiling despite a live 200.
   ASSERT(EffectiveManualCeiling(FP_FROMINT(200), FP_FROMINT(50)) == FP_FROMINT(50));
   // Operator saved manualiqmax=200 and power-cycled -> latch re-taken at 200.
   const int32_t reLatched = FP_FROMINT(200);
   ASSERT(EffectiveManualCeiling(FP_FROMINT(200), reLatched) == FP_FROMINT(200));
   ASSERT(ClampManualCurrent(FP_FROMINT(150),
             EffectiveManualCeiling(FP_FROMINT(200), reLatched)) == FP_FROMINT(150));
}

/* The shared guard that keeps the ceiling in range: Param::Set rejects a value
 * outside [min,max] and leaves the stored value untouched, accepting in-range
 * writes. manualiqmax is FOC-only so it isn't in the SINE param branch the host
 * links (its bounds are firmware-compile-checked); this pins the identical
 * mechanism on ampnom (SINE, range 0..100) — the guard that stops a host
 * writing manualiqmax past 400 or negative. */
static void TestParamSetRejectsOutOfRange()
{
   Param::SetInt(Param::ampnom, 50); // known in-range starting point

   // Above max: rejected, value unchanged.
   ASSERT(Param::Set(Param::ampnom, FP_FROMINT(101)) == -1);
   ASSERT(Param::GetInt(Param::ampnom) == 50);

   // Below min: rejected, value unchanged.
   ASSERT(Param::Set(Param::ampnom, FP_FROMINT(-1)) == -1);
   ASSERT(Param::GetInt(Param::ampnom) == 50);

   // In-range: accepted and stored.
   ASSERT(Param::Set(Param::ampnom, FP_FROMINT(80)) == 0);
   ASSERT(Param::GetInt(Param::ampnom) == 80);

   Param::SetInt(Param::ampnom, 0); // restore default; don't leak to other suites
}

REGISTER_TEST(ManualClampTest, TestDefaultMaxIsNoOp, TestClampsIqBothSigns,
              TestClampsIdBothSigns, TestNegativeCeilingIsDefensive,
              TestLatchBlocksRuntimeRaise, TestLatchAllowsRuntimeLower,
              TestReLatchAppliesRaise, TestParamSetRejectsOutOfRange);
