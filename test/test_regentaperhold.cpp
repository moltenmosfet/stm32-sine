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

/* T6 (F4): the regen-taper hold policy extracted into RegenTaperHold
 * (include/regentaperhold.h). These exercise it in isolation from
 * VehicleControl::ProcessThrottle() / Param::. HOLD_CALLS is 50 (500 ms at
 * the 100 Hz ProcessThrottle rate). */

#include "regentaperhold.h"
#include "test.h"
#include <cmath>

static const float BRKRAMPSTR = 10.0f; // Hz
static const int HOLD_CALLS = 50;

static bool NearlyEqual(float a, float b)
{
   return std::fabs(a - b) < 1e-5f;
}

class RegenTaperHoldTest: public UnitTest
{
   public:
      RegenTaperHoldTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

/* Sequence: taper at 0.4 -> freq snaps to 0 -> factor holds 0.4 for
 * HOLD_CALLS calls -> releases to 0 once the hold expires. */
static void TestHoldsLastFactorThenReleases()
{
   RegenTaperHold hold;
   const float finalSpnt = -100.0f;
   const float freqAt0p4 = 0.4f * BRKRAMPSTR;

   float out = hold.Apply(freqAt0p4, BRKRAMPSTR, finalSpnt);
   ASSERT(NearlyEqual(out, 0.4f * finalSpnt));

   // Freq snaps to exactly 0 (below the encoder deadband): must hold 0.4
   // for exactly HOLD_CALLS calls.
   for (int i = 0; i < HOLD_CALLS; i++)
   {
      out = hold.Apply(0.0f, BRKRAMPSTR, finalSpnt);
      ASSERT(NearlyEqual(out, 0.4f * finalSpnt));
   }

   // The hold has now expired: release to zero.
   out = hold.Apply(0.0f, BRKRAMPSTR, finalSpnt);
   ASSERT(NearlyEqual(out, 0.0f));

   // Stays released on further zero-freq calls.
   out = hold.Apply(0.0f, BRKRAMPSTR, finalSpnt);
   ASSERT(NearlyEqual(out, 0.0f));
}

/* Frequency recovering to nonzero mid-hold must resume the normal taper
 * immediately (not the stale held value) and refresh the hold window. */
static void TestRecoveryMidHoldResumesNormalTaper()
{
   RegenTaperHold hold;
   const float finalSpnt = -100.0f;
   const float freqAt0p4 = 0.4f * BRKRAMPSTR;
   const float freqAt0p7 = 0.7f * BRKRAMPSTR;

   hold.Apply(freqAt0p4, BRKRAMPSTR, finalSpnt);

   // Hold partway through (well short of HOLD_CALLS).
   for (int i = 0; i < 10; i++)
      hold.Apply(0.0f, BRKRAMPSTR, finalSpnt);

   // Frequency recovers: taper must use the live value, not the held 0.4.
   float out = hold.Apply(freqAt0p7, BRKRAMPSTR, finalSpnt);
   ASSERT(NearlyEqual(out, 0.7f * finalSpnt));

   // Hold window must have been refreshed: a fresh HOLD_CALLS zero-freq run
   // must still return the newly-held 0.7 factor throughout.
   for (int i = 0; i < HOLD_CALLS; i++)
   {
      out = hold.Apply(0.0f, BRKRAMPSTR, finalSpnt);
      ASSERT(NearlyEqual(out, 0.7f * finalSpnt));
   }

   out = hold.Apply(0.0f, BRKRAMPSTR, finalSpnt);
   ASSERT(NearlyEqual(out, 0.0f));
}

/* Positive (non-regen) finalSpnt must pass through unaffected, regardless
 * of rotorfreq/brkrampstr, and must not perturb hold state used by a later
 * regen event. */
static void TestPositiveFinalSpntUnaffected()
{
   RegenTaperHold hold;

   ASSERT(hold.Apply(0.0f, BRKRAMPSTR, 55.0f) == 55.0f);
   ASSERT(hold.Apply(0.4f * BRKRAMPSTR, BRKRAMPSTR, 55.0f) == 55.0f);
   ASSERT(hold.Apply(2 * BRKRAMPSTR, BRKRAMPSTR, 55.0f) == 55.0f);

   // A subsequent regen event starts clean: a full hold window is
   // available (positive-finalSpnt calls above did not consume it).
   const float finalSpnt = -100.0f;
   hold.Apply(0.4f * BRKRAMPSTR, BRKRAMPSTR, finalSpnt);
   for (int i = 0; i < HOLD_CALLS; i++)
   {
      float out = hold.Apply(0.0f, BRKRAMPSTR, finalSpnt);
      ASSERT(NearlyEqual(out, 0.4f * finalSpnt));
   }
}

/* rotorfreq >= brkrampstr (full regen, outside the taper band) must pass
 * finalSpnt through unchanged, same as the original ungated multiply. */
static void TestAboveThresholdUnaffected()
{
   RegenTaperHold hold;
   const float finalSpnt = -42.0f;

   float out = hold.Apply(BRKRAMPSTR, BRKRAMPSTR, finalSpnt);
   ASSERT(NearlyEqual(out, finalSpnt));

   out = hold.Apply(2 * BRKRAMPSTR, BRKRAMPSTR, finalSpnt);
   ASSERT(NearlyEqual(out, finalSpnt));
}

REGISTER_TEST(RegenTaperHoldTest, TestHoldsLastFactorThenReleases,
              TestRecoveryMidHoldResumesNormalTaper, TestPositiveFinalSpntUnaffected,
              TestAboveThresholdUnaffected);
