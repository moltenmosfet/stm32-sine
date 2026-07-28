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

/* G8b [FORK]: ClampThrottleCurrent (include/throtclamp.h) is the firmware-side
 * authority cap on the THROTTLE current path — the scope G8 explicitly left
 * open — extracted pure so the host harness drives the real clamp. In
 * pwmgeneration-foc.cpp it is applied once per SetTorquePercent call:
 *
 *     float is = Param::GetFloat(Param::throtcur) * torquePercent;
 *     is = ClampThrottleCurrent(is,
 *             EffectiveThrottleCeiling(Param::GetFloat(Param::throtcurmax),
 *                                      throtCurMaxLatch));
 *     FOC::Mtpa(is, id, iq);
 *
 * The seam is BEFORE Mtpa on purpose: Mtpa is magnitude-preserving
 * (idref^2 + iqref^2 == is^2), so bounding |is| bounds the stator current that
 * actually reaches the controllers. These cases prove: the 1000 A default is a
 * pure pass-through (no behaviour change until configured down, including at
 * the pre-G8b worst case throtcur=10 A/% x 100% throttle); the clamp bounds
 * BOTH polarities (drive and regen) once configured down; normal in-window
 * commands are untouched; and the clamp is defensive against an out-of-range
 * (negative) ceiling.
 *
 * The ceiling itself is raise-proofed at runtime by EffectiveThrottleCeiling =
 * MIN(live param, boot-latched value): the host can lower the cap live but a
 * runtime raise only takes hold after set+flash-save+reboot re-latches. Those
 * cases drive the pure guard directly with the latch injected.
 *
 * The `throtcurmax` param is a persisted PARAM_ENTRY (TYPE_PARAM) — it survives
 * a power cycle, which is what makes a commissioned ceiling durable rather than
 * resetting to the 1000 default every boot. It is FOC-only (it lives in
 * THROTTLE_PARAMETERS_FOC), so — like manualiqmax/manualiq/iqtimeout — it does
 * not exist in the SINE param branch the host suite links (its own
 * min/max/default are firmware-compile-checked instead). TestParamSetRejects
 * pins the shared range guard on a bounded param that IS in the SINE build, so
 * a regression that stopped Param::Set enforcing ranges (the guard that stops a
 * host writing throtcurmax past 1000) fails here.
 *
 * All values here are exactly representable in binary floating point and the
 * clamp returns one of its operands verbatim, so exact == comparison is sound. */

#include "throtclamp.h"
#include "params.h"
#include "my_math.h"
#include "test.h"

class ThrotClampTest: public UnitTest
{
   public:
      ThrotClampTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

static const float MAX1000 = 1000.0f; // default throtcurmax

/* Default ceiling (1000 A) is a no-op: any reachable command, either sign,
 * passes through unchanged — the shipped-default binary is behaviourally
 * identical to the pre-G8b firmware. 1000 A is the full reachable range
 * (throtcur max 10 A/% x throttle magnitude 100%), so nothing can bind it. */
static void TestDefaultMaxIsNoOp()
{
   ASSERT(ClampThrottleCurrent(0.0f,     MAX1000) == 0.0f);
   ASSERT(ClampThrottleCurrent(150.0f,   MAX1000) == 150.0f);
   ASSERT(ClampThrottleCurrent(-150.0f,  MAX1000) == -150.0f);
   ASSERT(ClampThrottleCurrent(1000.0f,  MAX1000) == 1000.0f);  // boundary
   ASSERT(ClampThrottleCurrent(-1000.0f, MAX1000) == -1000.0f); // boundary
}

/* Configured down, the clamp bounds the throttle current magnitude on BOTH
 * polarities — drive and regen are symmetric because maxMag is a magnitude.
 * In-window commands still pass through untouched. */
static void TestClampsBothSigns()
{
   const float cap = 50.0f;

   ASSERT(ClampThrottleCurrent(150.0f,  cap) == 50.0f);  // drive over -> +cap
   ASSERT(ClampThrottleCurrent(-150.0f, cap) == -50.0f); // regen over -> -cap
   ASSERT(ClampThrottleCurrent(50.0f,   cap) == 50.0f);  // at cap, untouched
   ASSERT(ClampThrottleCurrent(-50.0f,  cap) == -50.0f);
   ASSERT(ClampThrottleCurrent(30.0f,   cap) == 30.0f);  // in-window, untouched
   ASSERT(ClampThrottleCurrent(-30.0f,  cap) == -30.0f);
}

/* The real arithmetic of the seam: is = throtcur [A/%] * torquePercent [%].
 * Pins the gap G8b closes — at the param maxima the pre-G8b firmware would
 * command 1000 A (2.5x the manual path's +/-400 A range that G8 capped) — and
 * shows a commissioned ceiling binding it on both polarities while ordinary
 * throttle operation is unaffected. */
static void TestCapsThrotcurScaledRequest()
{
   // Worst case reachable before G8b: throtcur at its param max, full throttle.
   const float worstCase = 10.0f * 100.0f; // 1000 A
   ASSERT(ClampThrottleCurrent(worstCase, MAX1000) == 1000.0f); // default: unchanged

   // Commissioned down to a bench-safe 120 A: bound on drive AND regen.
   const float cap = 120.0f;
   ASSERT(ClampThrottleCurrent(worstCase,  cap) == 120.0f);
   ASSERT(ClampThrottleCurrent(-worstCase, cap) == -120.0f);

   // Ordinary operation (throtcur 1 A/%, 80% throttle) is untouched by the cap.
   ASSERT(ClampThrottleCurrent(1.0f * 80.0f, cap) == 80.0f);
   ASSERT(ClampThrottleCurrent(1.0f * -80.0f, cap) == -80.0f);

   // A zero ceiling fully disables throttle current authority.
   ASSERT(ClampThrottleCurrent(worstCase,  0.0f) == 0.0f);
   ASSERT(ClampThrottleCurrent(-worstCase, 0.0f) == 0.0f);
}

/* A negative ceiling can't come from the range-guarded param (min=0), but the
 * clamp must still be self-defensive: a negative maxMag is treated as 0 (full
 * lockout), never as an inverted window that would pass everything. */
static void TestNegativeCeilingIsDefensive()
{
   ASSERT(ClampThrottleCurrent(200.0f,  -10.0f) == 0.0f);
   ASSERT(ClampThrottleCurrent(-200.0f, -10.0f) == 0.0f);
   ASSERT(ClampThrottleCurrent(0.0f,    -10.0f) == 0.0f);
}

/* Boot latch: a runtime attempt to RAISE the ceiling over CAN has no live
 * effect — the effective ceiling stays pinned at the boot-latched value, so a
 * host bug cannot restore authority it wasn't commissioned with. */
static void TestLatchBlocksRuntimeRaise()
{
   const float latched = 120.0f; // commissioned + flash-loaded ceiling

   // Host writes throtcurmax up to 1000 at runtime: effective stays 120.
   ASSERT(EffectiveThrottleCeiling(1000.0f, latched) == 120.0f);
   // ...and a command is still clamped to the latched ceiling, not the raise.
   ASSERT(ClampThrottleCurrent(800.0f,
             EffectiveThrottleCeiling(1000.0f, latched)) == 120.0f);
}

/* A runtime LOWER takes effect immediately: MIN picks the lower live value, so
 * an operator/host can always de-rate authority without a reboot. */
static void TestLatchAllowsRuntimeLower()
{
   const float latched = 120.0f;

   ASSERT(EffectiveThrottleCeiling(40.0f, latched) == 40.0f);
   ASSERT(ClampThrottleCurrent(800.0f,
             EffectiveThrottleCeiling(40.0f, latched)) == 40.0f);
   // Equal live/latched (fresh board at default, or just-re-latched): pass-through.
   ASSERT(EffectiveThrottleCeiling(1000.0f, 1000.0f) == 1000.0f);
}

/* After set+save+reboot the latch is re-taken at the new (higher) value; from
 * then on the raised ceiling is live. Modelled by latching at the new value. */
static void TestReLatchAppliesRaise()
{
   // Pre-reboot: latched 120 pins the effective ceiling despite a live 400.
   ASSERT(EffectiveThrottleCeiling(400.0f, 120.0f) == 120.0f);
   // Operator saved throtcurmax=400 and power-cycled -> latch re-taken at 400.
   const float reLatched = 400.0f;
   ASSERT(EffectiveThrottleCeiling(400.0f, reLatched) == 400.0f);
   ASSERT(ClampThrottleCurrent(300.0f,
             EffectiveThrottleCeiling(400.0f, reLatched)) == 300.0f);
}

/* The shared guard that keeps the ceiling in range: Param::Set rejects a value
 * outside [min,max] and leaves the stored value untouched, accepting in-range
 * writes. throtcurmax is FOC-only so it isn't in the SINE param branch the host
 * links (its bounds are firmware-compile-checked); this pins the identical
 * mechanism on ampnom (SINE, range 0..100) — the guard that stops a host
 * writing throtcurmax past 1000 or negative. */
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

REGISTER_TEST(ThrotClampTest, TestDefaultMaxIsNoOp, TestClampsBothSigns,
              TestCapsThrotcurScaledRequest, TestNegativeCeilingIsDefensive,
              TestLatchBlocksRuntimeRaise, TestLatchAllowsRuntimeLower,
              TestReLatchAppliesRaise, TestParamSetRejectsOutOfRange);
