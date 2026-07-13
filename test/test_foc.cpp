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

/* Baseline characterisation of the FOC transforms and helpers. foc.cpp does its
 * modulation math in Q15 internally; these tests treat modulation/current
 * values as raw digits and lean on transform invariants rather than exact
 * bit patterns, so they stay meaningful across the T3 GetQLimit change. */

#include "foc.h"
#include "test.h"
#include <cstdint>
#include <cmath>
#include <cstdlib>

class FocTest: public UnitTest
{
   public:
      FocTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

/* |a-b| within pct% of the larger magnitude. */
static bool WithinPercent(int64_t a, int64_t b, int pct)
{
   int64_t ref = std::max<int64_t>(std::llabs(a), std::llabs(b));
   return std::llabs(a - b) * 100 <= (int64_t)pct * ref;
}

/* GetQLimit on the two valid edges:
 *   ud = 0      -> sqrt(modMaxPow2)      = modMax
 *   ud = modMax -> sqrt(modMaxPow2-max^2)= 0
 * The negative-radicand (ud^2 > modMaxPow2) overflow is F6/T3's test, not here. */
static void TestGetQLimitEdges()
{
   int32_t modMax = FOC::GetMaximumModulationIndex();

   ASSERT(std::abs(FOC::GetQLimit(0) - modMax) <= 2); // integer Newton, allow +/-1
   ASSERT(FOC::GetQLimit(modMax) == 0);
}

/* F6/T3: ud beyond modMax makes modMaxPow2 - ud*ud negative. The unsigned sqrt
 * then reads it as a huge uint32 and returns a garbage q-limit (~65k),
 * commanding overmodulation. The fix clamps the radicand at 0. */
static void TestGetQLimitRadicandClamp()
{
   int32_t modMax = FOC::GetMaximumModulationIndex();
   ASSERT(FOC::GetQLimit(modMax + 1000) == 0); // master: ~65k
}

/* ParkClarke preserves magnitude regardless of rotor angle:
 *   id^2 + iq^2 = ia^2 + ib^2 for all angles (Park is a rotation). */
static void TestParkClarkeMagnitudeInvariance()
{
   const s32fp il1 = 2000, il2 = -800;

   const uint16_t angles[] = { 0, 12000, 27000, 50000 };
   int64_t magFirst = 0;

   for (int i = 0; i < 4; i++)
   {
      FOC::SetAngle(angles[i]);
      FOC::ParkClarke(il1, il2);
      int64_t mag = (int64_t)FOC::id * FOC::id + (int64_t)FOC::iq * FOC::iq;

      if (i == 0)
         magFirst = mag;
      else
         ASSERT(WithinPercent(mag, magFirst, 2)); // rotation invariance, FP slop
   }

   ASSERT(magFirst > 0); // sanity: transform actually produced current
}

/* InvParkClarke: the phase-to-phase (line) voltages are common-mode invariant,
 * so the sum of squared line differences is angle-independent for a fixed |u|
 * (away from short-pulse suppression). */
static void TestInvParkClarkeLineVoltageInvariance()
{
   const int32_t ud = 5000, uq = 5000; // |u| ~7071, well inside modMax, no clamp

   auto lineEnergy = [](uint16_t angle) -> int64_t {
      FOC::SetAngle(angle);
      FOC::InvParkClarke(ud, uq);
      int64_t d01 = (int64_t)FOC::DutyCycles[0] - FOC::DutyCycles[1];
      int64_t d12 = (int64_t)FOC::DutyCycles[1] - FOC::DutyCycles[2];
      int64_t d20 = (int64_t)FOC::DutyCycles[2] - FOC::DutyCycles[0];
      return d01 * d01 + d12 * d12 + d20 * d20;
   };

   int64_t m0 = lineEnergy(0);
   ASSERT(m0 > 0);
   ASSERT(WithinPercent(lineEnergy(15000), m0, 2));
   ASSERT(WithinPercent(lineEnergy(40000), m0, 2));
}

/* Mtpa: id depends on |is| only (even), iq is odd in is, and the split conserves
 * total current: id^2 + iq^2 = is^2. */
static void TestMtpaSymmetryAndMagnitude()
{
   FOC::SetMotorParameters(0.0001f, 0.1f); // salient machine -> nonzero term1

   float idp, iqp, idn, iqn;
   FOC::Mtpa(100.0f, idp, iqp);
   FOC::Mtpa(-100.0f, idn, iqn);

   ASSERT(std::fabs(idp - idn) < 0.05f);        // id even in is
   ASSERT(std::fabs(iqp + iqn) < 0.05f);        // iq odd in is
   ASSERT(iqp > 0.0f && iqn < 0.0f);            // sign follows is

   float mag = idp * idp + iqp * iqp;
   ASSERT(WithinPercent((int64_t)mag, 10000, 1)); // is^2 = 100^2
}

/* F10/T5 dead-time compensation. ParkClarke captures phaseSign[0..2] from its
 * (il1, il2) arguments (il3 = -il1-il2); InvParkClarke adds/subtracts dtcomp
 * digits per phase accordingly. Currents here are the global Q5 s32fp (this
 * test file does not redefine CST_DIGITS the way foc.cpp does internally). */

/* Straight (non-swapped) calling convention: DutyCycles[0]/[1]/[2] follow the
 * sign of il1, il2, and -(il1+il2) respectively. */
static void TestDeadtimeCompSignMapping()
{
   const s32fp il1 = FP_FROMINT(10);  // +10 A -> phase0 positive
   const s32fp il2 = FP_FROMINT(3);   // +3 A  -> phase1 positive
   // il3 = -13 A -> phase2 negative
   const int32_t ud = 5000, uq = 3000; // inside modMax, no short-pulse clamp

   FOC::SetAngle(0);
   FOC::ParkClarke(il1, il2);

   FOC::InvParkClarke(ud, uq, 0);
   int32_t base0 = FOC::DutyCycles[0];
   int32_t base1 = FOC::DutyCycles[1];
   int32_t base2 = FOC::DutyCycles[2];

   FOC::InvParkClarke(ud, uq, 100);
   ASSERT(FOC::DutyCycles[0] - base0 == 100);
   ASSERT(FOC::DutyCycles[1] - base1 == 100);
   ASSERT(FOC::DutyCycles[2] - base2 == -100);
}

/* SWAP_CURRENTS calling convention: the caller (ProcessCurrents) resolves the
 * pinswap itself by swapping which measured current it passes as il1 vs il2.
 * ParkClarke has no pinswap awareness -- it just captures signs from argument
 * order -- so phaseSign[i] stays aligned with DutyCycles[i] as long as the
 * caller passed the swapped pair, which this test reproduces directly. */
static void TestDeadtimeCompRespectsPinswapCallingConvention()
{
   const s32fp measuredIl1 = FP_FROMINT(-10); // wired to duty-channel 1 here
   const s32fp measuredIl2 = FP_FROMINT(6);   // wired to duty-channel 0 here
   // il3 = -(measuredIl2 + measuredIl1) = +4 A -> phase2 positive
   const int32_t ud = 3000, uq = 1000;

   FOC::SetAngle(5000);
   FOC::ParkClarke(measuredIl2, measuredIl1); // SWAP_CURRENTS calling order

   FOC::InvParkClarke(ud, uq, 0);
   int32_t base0 = FOC::DutyCycles[0];
   int32_t base1 = FOC::DutyCycles[1];
   int32_t base2 = FOC::DutyCycles[2];

   FOC::InvParkClarke(ud, uq, 150);
   ASSERT(FOC::DutyCycles[0] - base0 == 150);   // follows measuredIl2 (+6 A)
   ASSERT(FOC::DutyCycles[1] - base1 == -150);  // follows measuredIl1 (-10 A)
   ASSERT(FOC::DutyCycles[2] - base2 == 150);   // follows il3 (+4 A)
}

/* All three currents inside the +-2 A deadband -> no compensation on any
 * phase regardless of dtcomp magnitude. */
static void TestDeadtimeCompDeadband()
{
   const s32fp il1 = FP_FROMFLT(0.5); // il3 = -1.0 A; all three within +-2 A
   const s32fp il2 = FP_FROMFLT(0.5);
   const int32_t ud = 4000, uq = 2000;

   FOC::SetAngle(20000);
   FOC::ParkClarke(il1, il2);

   FOC::InvParkClarke(ud, uq, 0);
   int32_t base0 = FOC::DutyCycles[0];
   int32_t base1 = FOC::DutyCycles[1];
   int32_t base2 = FOC::DutyCycles[2];

   FOC::InvParkClarke(ud, uq, 500);
   ASSERT(FOC::DutyCycles[0] == base0);
   ASSERT(FOC::DutyCycles[1] == base1);
   ASSERT(FOC::DutyCycles[2] == base2);
}

/* dtcomp == 0 (the param default) must leave duties byte-for-byte identical
 * to omitting the argument entirely (default parameter value). */
static void TestDeadtimeCompOffByDefaultLeavesDutiesUnchanged()
{
   const s32fp il1 = FP_FROMINT(10);
   const s32fp il2 = FP_FROMINT(-10);
   const int32_t ud = 4000, uq = 2000;

   FOC::SetAngle(10000);
   FOC::ParkClarke(il1, il2);

   FOC::InvParkClarke(ud, uq); // default dtcomp = 0
   int32_t d0 = FOC::DutyCycles[0];
   int32_t d1 = FOC::DutyCycles[1];
   int32_t d2 = FOC::DutyCycles[2];

   FOC::InvParkClarke(ud, uq, 0); // explicit 0
   ASSERT(FOC::DutyCycles[0] == d0);
   ASSERT(FOC::DutyCycles[1] == d1);
   ASSERT(FOC::DutyCycles[2] == d2);
}

REGISTER_TEST(FocTest, TestGetQLimitEdges, TestGetQLimitRadicandClamp,
              TestParkClarkeMagnitudeInvariance,
              TestInvParkClarkeLineVoltageInvariance, TestMtpaSymmetryAndMagnitude,
              TestDeadtimeCompSignMapping, TestDeadtimeCompRespectsPinswapCallingConvention,
              TestDeadtimeCompDeadband, TestDeadtimeCompOffByDefaultLeavesDutiesUnchanged);
