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

/* Baseline characterisation of PiControllerGeneric<s32fp,int32_t> (the s32fp
 * instantiation stm32-sine actually uses). These lock in current behaviour so
 * the T2 fix (64-bit I-term, windup overflow guard, float specialisation) can be
 * shown to change only what it intends. Fixed point here is Q5 (1.0 = 32), the
 * same context picontroller.cpp compiles in. */

#include "picontroller.h"
#include "my_fp.h"
#include "test.h"

/* Control ISR rate the real firmware runs the current loops at. */
static const int FREQ = 8789;

class PiControllerTest: public UnitTest
{
   public:
      PiControllerTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

/* Proportional path: with ki=0 the output is feedForward + FP_TOINT(err*kp),
 * independent of the integrator. kp is a raw multiplier scaled by FP_TOINT, so
 * kp=32 is unity gain in digits. */
static void TestProportional()
{
   PiController pi;
   pi.SetCallingFrequency(FREQ);
   pi.SetGains(32, 0);                 // kp=32 (unity), ki=0
   pi.SetMinMaxY(-1000000, 1000000);   // wide, so no clamping
   pi.ResetIntegrator();

   pi.SetRef(FP_FROMINT(10));          // ref = 320 digits
   ASSERT(pi.Run(0) == 320);           // err=320 -> FP_TOINT(320*32) = 320

   pi.SetProportionalGain(64);         // double the gain
   ASSERT(pi.Run(0) == 640);           // FP_TOINT(320*64) = 640
}

/* Output saturation at minY/maxY, both polarities. */
static void TestOutputClamp()
{
   PiController pi;
   pi.SetCallingFrequency(FREQ);
   pi.SetGains(32, 0);
   pi.SetMinMaxY(-50, 50);
   pi.ResetIntegrator();

   pi.SetRef(FP_FROMINT(10));          // raw output 320, clamps to +50
   ASSERT(pi.Run(0) == 50);

   pi.SetRef(-FP_FROMINT(10));         // raw output -320, clamps to -50
   ASSERT(pi.Run(0) == -50);
}

/* SetMinMaxY must recompute the anti-windup integrator bounds every call
 * (picontroller.h SetMinMaxY -> SetIntegralGain). With ki=20000 the bound is
 * well within int32, so no overflow (that overflow edge is T2's territory). */
static void TestWindupBoundsRecomputeOnSetMinMaxY()
{
   PiController pi;
   pi.SetCallingFrequency(FREQ);
   pi.SetGains(0, 20000);              // pure integral
   pi.ResetIntegrator();

   // maxSum = FP_FROMINT((37000*8789)/20000) = FP_FROMINT(16259) = 520288
   pi.SetMinMaxY(-37000, 37000);
   pi.SetRef(FP_FROMINT(100));         // constant +3200 error
   for (int i = 0; i < 400; i++)
      pi.Run(0);
   ASSERT(pi.GetIntegrator() == 520288);

   // Widen the range: bounds must be recomputed higher.
   // maxSum = FP_FROMINT((40000*8789)/20000) = FP_FROMINT(17578) = 562496
   pi.SetMinMaxY(-40000, 40000);
   for (int i = 0; i < 400; i++)
      pi.Run(0);
   ASSERT(pi.GetIntegrator() == 562496);
}

REGISTER_TEST(PiControllerTest, TestProportional, TestOutputClamp,
              TestWindupBoundsRecomputeOnSetMinMaxY);
