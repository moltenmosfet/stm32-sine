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

/* T10 (F15): Stm32Scheduler::Run() advances a channel's TIM_CCR by its period
 * every cycle, then calls the task. If that task took longer than one period,
 * the just-advanced compare value is already behind the counter; unresynced,
 * the channel would not fire again until the 16-bit counter wraps all the way
 * around (up to ~655 ms at 100 kHz).
 *
 * Run() itself reads/writes TIM_CCR through raw memory-mapped register
 * access (the TIM_CCR(t,c) macro dereferences &TIM_CCR1(timer) + c), which
 * needs a real hardware register address and can't be driven from the host
 * suite -- the same reason Stm32Can::ConfigureFilters stayed untested and
 * only its extracted pure packing logic (canfilterpack.cpp) got a test.
 * Stm32Scheduler::CheckOverrun is that same kind of extraction: the deadline
 * decision and resync/counter side effects on plain counter/compare values,
 * with no register access, so it is exercised directly here. Run()'s use of
 * it is a 4-line, review-verified wrapper. */

#include "stm32scheduler.h"
#include "test.h"
#include <cstdint>

class Stm32SchedulerTest: public UnitTest
{
   public:
      Stm32SchedulerTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

// Normal case: task finishes well inside its period. The += periods[i] line
// (modeled here by ccr already sitting one period ahead of the pre-task
// counter) must survive untouched -- next execution stays exactly one
// period after the previous compare value, no resync.
static void TestNoOverrunOnShortTask()
{
   uint32_t period = 1000;
   uint32_t ccr = period;       // TIM_CCR(timer,i) += periods[i] already applied
   uint32_t counterAfterTask = period / 2; // task finished well before ccr
   uint32_t newCcr = 0xdeadbeef;
   uint32_t overruns = 0;

   bool missed = Stm32Scheduler::CheckOverrun(ccr, counterAfterTask, period, newCcr, overruns);

   ASSERT(!missed);
   ASSERT(overruns == 0);
}

// Overrun case: task "takes" 1.5 periods -- by the time it returns, the
// counter has moved past the compare value the += line set up. The channel
// must be resynced to one period ahead of *now* (the counter), not left
// behind in the past.
static void TestOverrunResyncsAheadOfNow()
{
   uint32_t period = 1000;
   uint32_t ccr = period;                 // TIM_CCR(timer,i) += periods[i]
   uint32_t counterAfterTask = 3 * period / 2; // task ran 1.5x its period
   uint32_t newCcr = 0;
   uint32_t overruns = 0;

   bool missed = Stm32Scheduler::CheckOverrun(ccr, counterAfterTask, period, newCcr, overruns);

   ASSERT(missed);
   ASSERT(newCcr == counterAfterTask + period); // one period ahead of now
   ASSERT((int16_t)(newCcr - counterAfterTask) > 0); // ahead, not in the past
   ASSERT(overruns == 1);
}

// Wrap correctness: a compare value just past 0x0000 while the counter sits
// just below 0xFFFF is 26 ticks *ahead* across the wrap, not behind -- the
// signed 16-bit difference must read that correctly and not flag it as
// missed (master's naive unsigned compare would get this wrong at the wrap).
static void TestNoFalseOverrunAcrossWrap()
{
   uint32_t period = 100;
   uint32_t ccr = 10;            // just past the 0x0000 wrap
   uint32_t counterAfterTask = 0xFFF0; // just below 0xFFFF (16 ticks from wrap)
   uint32_t newCcr = 0xdeadbeef;
   uint32_t overruns = 0;

   bool missed = Stm32Scheduler::CheckOverrun(ccr, counterAfterTask, period, newCcr, overruns);

   ASSERT(!missed); // 26 ticks ahead across the wrap, not behind
   ASSERT(overruns == 0);
}

// The inverse: a genuine miss that straddles the wrap (ccr just below 0xFFFF,
// counter just past 0x0000) must still be caught.
static void TestOverrunDetectedAcrossWrap()
{
   uint32_t period = 100;
   uint32_t ccr = 0xFFF0;        // just below the wrap
   uint32_t counterAfterTask = 10; // just past 0x0000 -- ccr is 26 ticks behind
   uint32_t newCcr = 0;
   uint32_t overruns = 0;

   bool missed = Stm32Scheduler::CheckOverrun(ccr, counterAfterTask, period, newCcr, overruns);

   ASSERT(missed);
   ASSERT(newCcr == counterAfterTask + period);
   ASSERT(overruns == 1);
}

REGISTER_TEST(Stm32SchedulerTest, TestNoOverrunOnShortTask, TestOverrunResyncsAheadOfNow,
              TestNoFalseOverrunAcrossWrap, TestOverrunDetectedAcrossWrap);
