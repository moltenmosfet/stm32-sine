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

/* T11 (F5): AntiCogTriangle (include/anticog.h) folds a full electrical
 * revolution into a +-32767ish triangle. Master computed the final scale
 * as `uint16_t antiCog = 4 * angle`, which wraps 65536->0 right at the
 * 270 degree branch boundary (raw angle 49151->49152) and emits a
 * one-cycle glitch from the triangle peak (~32769) straight to its trough
 * (~-32767). This characterizes the fixed function across the entire
 * angle domain: adjacent raw angle inputs may never produce outputs that
 * differ by more than the per-step slope (4 digits/step). */

#include "anticog.h"
#include "test.h"
#include <cstdlib>

class AntiCogTest: public UnitTest
{
   public:
      AntiCogTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

static const int32_t MAX_STEP = 4; //slope of the triangle in the pre-scale domain

static void TestNoWrapGlitchAcrossFullRevolution()
{
   int32_t prev = AntiCogTriangle(0);

   for (uint32_t angle = 1; angle < 65536; angle++)
   {
      int32_t cur = AntiCogTriangle((uint16_t)angle);
      ASSERT(std::abs(cur - prev) <= MAX_STEP);
      prev = cur;
   }

   // The domain wraps (uint16_t angle 65535 -> 0); the triangle must stay
   // continuous across that wrap too.
   ASSERT(std::abs(AntiCogTriangle(0) - prev) <= MAX_STEP);
}

static void TestOldWrapPointNoLongerGlitches()
{
   // angle 49151 folds to 16384, where master's `4 * (uint16_t)16384`
   // wrapped to 0 and produced antiCogScaled == -32767 instead of the true
   // triangle peak. Confirm the peak survives and the adjacent sample
   // (49152) is a small step away, not a 65536-digit jump.
   int32_t peak = AntiCogTriangle(49151);
   int32_t next = AntiCogTriangle(49152);

   ASSERT(peak > 30000);
   ASSERT(std::abs(next - peak) <= MAX_STEP);
}

REGISTER_TEST(AntiCogTest, TestNoWrapGlitchAcrossFullRevolution,
              TestOldWrapPointNoLongerGlitches);
