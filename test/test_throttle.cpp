/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

#include "my_fp.h"
#include "my_math.h"
#include "test.h"
#include "throttle.h"

class ThrottleTest: public UnitTest
{
   public:
      ThrottleTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

static void TestSetup()
{
      Throttle::potmin[0] = 1000;
      Throttle::potmax[0] = 2000;
      Throttle::potmin[1] = 3000;
      Throttle::potmax[1] = 4000;
      Throttle::linearity = 0.999;
      Throttle::brknom = 30;
      Throttle::brknompedal = -60;
      Throttle::regenRamp = 25;
      Throttle::brkmax = -50;
      Throttle::idleSpeed = 100;
      Throttle::speedkp = FP_FROMFLT(0.25);
      Throttle::speedflt = 5;
      Throttle::idleThrotLim = FP_FROMFLT(30);
}

//CalcThrottle takes normalized percentages, not raw pot counts; pot2nom is
//the regen-strength pot. Ramping lives in RampThrottle (covered via
//test_vcu.cpp), not here.
static void TestBrkPedal()
{
   //Brake pedal forces brknompedal (-60) scaled by pot2nom, regardless of
   //throttle position; the -0.1 offset keeps it strictly negative
   float percent = Throttle::CalcThrottle(65, 100, true);
   ASSERT(ABS(percent - -60.1f) < 0.01f)
   percent = Throttle::CalcThrottle(65, 50, true);
   ASSERT(ABS(percent - -30.1f) < 0.01f)
   //pot2nom=0 must still never reach 0 -- that could spin up the motor
   percent = Throttle::CalcThrottle(65, 0, true);
   ASSERT(percent < 0 && percent > -1)
}

static void TestRegen()
{
   //Off-pedal regen zone (potnom < brknom): proportional from brkmax (-50)
   //at zero pedal to 0 at brknom
   float percent = Throttle::CalcThrottle(15, 100, false);
   ASSERT(ABS(percent - -25.05f) < 0.01f)
   percent = Throttle::CalcThrottle(30, 100, false);
   ASSERT(percent == 0)
   //Regen-strength pot scales the off-pedal zone too
   percent = Throttle::CalcThrottle(15, 50, false);
   ASSERT(ABS(percent - -12.55f) < 0.01f)
}

static void TestLinearity()
{
   float percent = Throttle::CalcThrottle(0, 100, false);
   ASSERT((int)percent == -50)
   percent = Throttle::CalcThrottle(100, 100, false);
   ASSERT((int)percent == 100)
   percent = Throttle::CalcThrottle(65, 100, false);
   ASSERT((int)percent == 49)
   Throttle::linearity = 0.5;
   percent = Throttle::CalcThrottle(100, 100, false);
   ASSERT((int)percent == 100)
   percent = Throttle::CalcThrottle(65, 100, false);
   ASSERT((int)percent == 37)
   Throttle::linearity = 0;
   percent = Throttle::CalcThrottle(100, 100, false);
   ASSERT((int)percent == 100)
   percent = Throttle::CalcThrottle(65, 100, false);
   ASSERT((int)percent == 25)
}

// T12/F16: FrequencyLimitCommand used to keep its IIR filter state in a
// function-local static shared by two callers per Ms10Task cycle
// (vehiclecontrol.cpp: the final torque command, and the field-weakening
// current derate), so the filter ran twice as fast as its tuned time
// constant. FrequencyLimitCommand and the new FrequencyLimitCommandFw now
// own separate state. This test drives each filter to convergence with a
// different target frequency and checks that driving one does not perturb
// the other -- on master (shared static), the second block's calls would
// have dragged the first filter's state toward 200 as well.
static void TestFrequencyLimitStateIsolated()
{
   const int settleIterations = 200; // (15/16)^200 ~= 1e-5, plenty settled
   Throttle::fmax = 100;

   for (int i = 0; i < settleIterations; i++)
   {
      float spnt = 1000;
      Throttle::FrequencyLimitCommand(spnt, 50);
   }

   for (int i = 0; i < settleIterations; i++)
   {
      float fwSpnt = 1000;
      Throttle::FrequencyLimitCommandFw(fwSpnt, 200);
   }

   //One more primary-caller update at its already-converged frequency (50):
   //if state were still shared with the Fw caller, this would see a filtered
   //value near 200 instead of 50, and clamp res to ~0 instead of ~200.
   float spnt = 1000;
   Throttle::FrequencyLimitCommand(spnt, 50);
   ASSERT(ABS(spnt - 200.0f) < 0.01f)

   //And the Fw caller must likewise still see its own converged value (200),
   //unaffected by the primary caller's interleaved calls above.
   float fwSpnt = 1000;
   Throttle::FrequencyLimitCommandFw(fwSpnt, 200);
   ASSERT(fwSpnt == 0)
}

//This line registers the test
REGISTER_TEST(ThrottleTest, TestSetup, TestBrkPedal, TestRegen, TestLinearity, TestFrequencyLimitStateIsolated);
