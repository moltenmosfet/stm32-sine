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

/* T15 (F17): demonstrate that GetIlMax's naive sum-of-squares overflows int32
 * at high phase currents while fp_hypot3 (issue #29's helper) stays correct.
 * GetIlMax lives in pwmgeneration-sine.cpp, which is not in the host build, so
 * this replicates its math on my_fp (which is). Currents are Q5 (1 A = 32). */

#include "my_fp.h"
#include "my_math.h"
#include "test.h"
#include <cstdlib>

#define INV_SQRT_1_5 FP_FROMFLT(0.8164965809) // matches pwmgeneration-sine.cpp

class GetIlMaxTest: public UnitTest
{
   public:
      GetIlMaxTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

// Old GetIlMax math: FP_MUL(il,il) overflows int32 above ~1500 A peak.
static s32fp OldIlMax(s32fp il1, s32fp il2)
{
   s32fp il3 = -il1 - il2;
   s32fp ilMax = FP_MUL(il1, il1) + FP_MUL(il2, il2) + FP_MUL(il3, il3);
   ilMax = fp_sqrt(ilMax);
   return FP_MUL(ilMax, INV_SQRT_1_5);
}

// New GetIlMax math: fp_hypot3 scales down to avoid the overflow.
static s32fp NewIlMax(s32fp il1, s32fp il2)
{
   s32fp il3 = -il1 - il2;
   return FP_MUL(fp_hypot3(il1, il2, il3), INV_SQRT_1_5);
}

// At currents that don't overflow, old and new must agree (fp_hypot3's scaling
// loop is a no-op below 16383, so the computation is bit-identical).
static void TestIlMaxLowCurrentMatches()
{
   s32fp il1 = 100 * 32, il2 = -50 * 32; // balanced 100 A peak
   ASSERT(std::abs(OldIlMax(il1, il2) - NewIlMax(il1, il2)) <= 1);
}

// At 1500 A peak the naive squares overflow (sum goes negative -> garbage
// through the unsigned sqrt). fp_hypot3 stays positive and linear.
static void TestIlMaxHighCurrentOverflow()
{
   s32fp il1 = 1500 * 32, il2 = -750 * 32; // balanced 1500 A peak
   s32fp il3 = -il1 - il2;

   int32_t naiveSum = FP_MUL(il1, il1) + FP_MUL(il2, il2) + FP_MUL(il3, il3);
   ASSERT(naiveSum < 0); // the overflow the fix removes

   s32fp new100 = NewIlMax(100 * 32, -50 * 32);
   s32fp new1500 = NewIlMax(il1, il2);
   ASSERT(new1500 > 0);
   // ilMax is linear in current; 1500 A is 15x the 100 A point.
   ASSERT(std::abs(new1500 - 15 * new100) * 100 <= 3 * (15 * new100));
}

REGISTER_TEST(GetIlMaxTest, TestIlMaxLowCurrentMatches, TestIlMaxHighCurrentOverflow);
