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

/* T4 (F1): the low-speed q-clamp policy extracted into QClamp
 * (include/qclamp.h). These exercise the hysteresis + slew logic in
 * isolation from PwmGeneration::Run() / Param::. Frequencies are s32fp Q5
 * (FP_FROMINT(hz)); qlimit is a modulation-digit magnitude, matching the
 * ~37836 full-scale value foc.cpp works with (facts doc, MOTOR_PARAMETERS_FOC
 * modmax default). */

#include "qclamp.h"
#include "my_fp.h"
#include "test.h"
#include <cstdlib>

static const s32fp THRESHOLD = FP_FROMINT(30);
static const int32_t QLIMIT = 37836;
static const int32_t MAX_STEP = QLIMIT / 256; // matches QClamp's slew divisor

class QClampTest: public UnitTest
{
   public:
      QClampTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

/* (a) Dithering +-1 Hz around the 30 Hz threshold must never cross the +-2 Hz
 * hysteresis band (28..32 Hz), so the clamp must not oscillate between the
 * restricted and full-range targets. Once the slew has converged, further
 * dithering must leave the bounds exactly where they are. */
static void TestNoOscillationOnDitherFromRestricted()
{
   QClamp clamp; // starts restricted
   int dir = 1;

   // Settle deep into the restricted target first (frq well below threshold).
   for (int i = 0; i < 400; i++)
      clamp.Update(FP_FROMINT(20), QLIMIT, dir, THRESHOLD);

   ASSERT(clamp.GetMinLim() == 0);
   ASSERT(clamp.GetMaxLim() == QLIMIT);

   // Dither 29 <-> 31 Hz: inside the 28..32 Hz hysteresis band, so the
   // restricted state (and therefore the converged bounds) must not move.
   for (int i = 0; i < 40; i++)
   {
      s32fp frq = (i % 2 == 0) ? FP_FROMINT(29) : FP_FROMINT(31);
      clamp.Update(frq, QLIMIT, dir, THRESHOLD);
      ASSERT(clamp.GetMinLim() == 0);
      ASSERT(clamp.GetMaxLim() == QLIMIT);
   }
}

static void TestNoOscillationOnDitherFromUnrestricted()
{
   QClamp clamp;
   int dir = 1;

   // Settle deep into the unrestricted (full range) target first.
   for (int i = 0; i < 400; i++)
      clamp.Update(FP_FROMINT(50), QLIMIT, dir, THRESHOLD);

   ASSERT(clamp.GetMinLim() == -QLIMIT);
   ASSERT(clamp.GetMaxLim() == QLIMIT);

   // Same 29 <-> 31 Hz dither: still inside the band, must stay unrestricted.
   for (int i = 0; i < 40; i++)
   {
      s32fp frq = (i % 2 == 0) ? FP_FROMINT(29) : FP_FROMINT(31);
      clamp.Update(frq, QLIMIT, dir, THRESHOLD);
      ASSERT(clamp.GetMinLim() == -QLIMIT);
      ASSERT(clamp.GetMaxLim() == QLIMIT);
   }
}

/* (b) Bound change rate must never exceed qlimit/256 per Update() call, in
 * either direction, across a full restricted<->unrestricted transition and
 * across a dir flip while restricted (both make the target jump instantly). */
static void TestSlewRateNeverExceedsLimit()
{
   QClamp clamp;
   int32_t prevMin = clamp.GetMinLim();
   int32_t prevMax = clamp.GetMaxLim();

   // Force restricted -> unrestricted by running frequency well above threshold.
   for (int i = 0; i < 400; i++)
   {
      clamp.Update(FP_FROMINT(60), QLIMIT, 1, THRESHOLD);
      ASSERT(std::abs(clamp.GetMinLim() - prevMin) <= MAX_STEP);
      ASSERT(std::abs(clamp.GetMaxLim() - prevMax) <= MAX_STEP);
      prevMin = clamp.GetMinLim();
      prevMax = clamp.GetMaxLim();
   }
   ASSERT(clamp.GetMinLim() == -QLIMIT);
   ASSERT(clamp.GetMaxLim() == QLIMIT);

   // Force unrestricted -> restricted by dropping well below threshold.
   for (int i = 0; i < 400; i++)
   {
      clamp.Update(FP_FROMINT(5), QLIMIT, 1, THRESHOLD);
      ASSERT(std::abs(clamp.GetMinLim() - prevMin) <= MAX_STEP);
      ASSERT(std::abs(clamp.GetMaxLim() - prevMax) <= MAX_STEP);
      prevMin = clamp.GetMinLim();
      prevMax = clamp.GetMaxLim();
   }
   ASSERT(clamp.GetMinLim() == 0);
   ASSERT(clamp.GetMaxLim() == QLIMIT);

   // Flip direction while still restricted: target swings from [0,qlimit]
   // to [-qlimit,0] in one call; the walked bounds must still obey the slew.
   for (int i = 0; i < 400; i++)
   {
      clamp.Update(FP_FROMINT(5), QLIMIT, -1, THRESHOLD);
      ASSERT(std::abs(clamp.GetMinLim() - prevMin) <= MAX_STEP);
      ASSERT(std::abs(clamp.GetMaxLim() - prevMax) <= MAX_STEP);
      prevMin = clamp.GetMinLim();
      prevMax = clamp.GetMaxLim();
   }
   ASSERT(clamp.GetMinLim() == -QLIMIT);
   ASSERT(clamp.GetMaxLim() == 0);
}

/* qlimit/256 is the exact per-call step: a single call off a converged
 * restricted bound must move by precisely that much, not more, not less. */
static void TestSlewStepIsExact()
{
   QClamp clamp;

   for (int i = 0; i < 400; i++)
      clamp.Update(FP_FROMINT(60), QLIMIT, 1, THRESHOLD); // converge unrestricted

   ASSERT(clamp.GetMinLim() == -QLIMIT);

   clamp.Update(FP_FROMINT(5), QLIMIT, 1, THRESHOLD); // one restricting step
   ASSERT(clamp.GetMinLim() == -QLIMIT + MAX_STEP);
}

/* (c) qlimfrq=0 (thresholdHz == 0) must disable the restriction entirely:
 * the bounds always walk to and settle at +-qlimit, even at zero frequency
 * and even starting from the restricted default. */
static void TestZeroThresholdAlwaysFullRange()
{
   QClamp clamp; // starts restricted
   int dir = 1;

   for (int i = 0; i < 400; i++)
      clamp.Update(0, QLIMIT, dir, 0); // frq == 0, thresholdHz == 0 -> disabled

   ASSERT(clamp.GetMinLim() == -QLIMIT);
   ASSERT(clamp.GetMaxLim() == QLIMIT);

   // Stays full range even while dithering low frequencies and reversing dir.
   for (int i = 0; i < 40; i++)
   {
      s32fp frq = (i % 2 == 0) ? FP_FROMINT(0) : FP_FROMINT(5);
      int d = (i % 2 == 0) ? 1 : -1;
      clamp.Update(frq, QLIMIT, d, 0);
      ASSERT(clamp.GetMinLim() == -QLIMIT);
      ASSERT(clamp.GetMaxLim() == QLIMIT);
   }
}

REGISTER_TEST(QClampTest, TestNoOscillationOnDitherFromRestricted,
              TestNoOscillationOnDitherFromUnrestricted, TestSlewRateNeverExceedsLimit,
              TestSlewStepIsExact, TestZeroThresholdAlwaysFullRange);
