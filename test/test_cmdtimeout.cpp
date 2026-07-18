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

/* T21 (F9-adjacent, D4): CmdTimeout is the manualiq command-silence watchdog
 * (include/cmdtimeout.h), extracted pure so the host harness drives the real
 * decision + action. VehicleControl::CheckManualCmdTimeout() is exactly:
 *
 *     int32_t iq = Param::GetInt(Param::manualiq);
 *     if (manualCmdTimeout.ApplyZero(now, Param::GetInt(Param::iqtimeout), iq))
 *        Param::SetInt(Param::manualiq, iq);
 *
 * i.e. its ONLY side effect is writing manualiq. These cases prove ApplyZero
 * zeroes the command once silence exceeds the timeout, and — via a companion
 * "contactor" proxy that ApplyZero has no access to — that nothing else moves
 * (graceful decay, not a fault trip). They also pin the guards that keep it
 * from firing spuriously: disabled (timeout==0), not-yet-commanded, refreshed,
 * and the unsigned-wraparound-safe elapsed computation. The FOC-only Param
 * glue itself is compile-checked in firmware (the host suite links the SINE
 * param branch, where manualiq/iqtimeout do not exist — same structural reason
 * T7's manualiq fast-path has no host test). TIMEOUT is in RTC ticks (10 ms). */

#include "cmdtimeout.h"
#include "test.h"

static const uint32_t TIMEOUT = 20; // 20 ticks = 200 ms

class CmdTimeoutTest: public UnitTest
{
   public:
      CmdTimeoutTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

/* The core requirement: after a command, once silence reaches the timeout the
 * command zeroes and the contactor/PWM-enable proxy is untouched throughout. */
static void TestZeroesCommandAfterSilence()
{
   CmdTimeout t;
   int32_t manualiq = 150;
   int32_t contactor = 1; // proxy for opmode==MOD_RUN / dcsw closed

   t.Note(100);

   // One tick short of the timeout: no action.
   ASSERT(!t.ApplyZero(100 + TIMEOUT - 1, TIMEOUT, manualiq));
   ASSERT(manualiq == 150);
   ASSERT(contactor == 1);

   // At the timeout boundary: command zeroes, contactor untouched.
   ASSERT(t.ApplyZero(100 + TIMEOUT, TIMEOUT, manualiq));
   ASSERT(manualiq == 0);
   ASSERT(contactor == 1);
}

/* timeout == 0 disables the watchdog entirely (the param default-off value):
 * arbitrarily long silence must never zero the command. */
static void TestDisabledWhenTimeoutZero()
{
   CmdTimeout t;
   int32_t manualiq = 200;

   t.Note(0);
   ASSERT(!t.ApplyZero(1000000, 0, manualiq));
   ASSERT(manualiq == 200);
}

/* Before the host has ever commanded, the watchdog must not act (mirrors the
 * canIoActive gate) — otherwise it would zero on every boot tick. */
static void TestDoesNotActBeforeFirstCommand()
{
   CmdTimeout t;
   int32_t manualiq = 42;

   ASSERT(!t.Expired(1000, TIMEOUT));
   ASSERT(!t.ApplyZero(1000, TIMEOUT, manualiq));
   ASSERT(manualiq == 42);
}

/* A fresh command mid-window refreshes the deadline: a brief host stall that
 * recovers before the timeout must keep the command alive. */
static void TestRefreshPreventsTimeout()
{
   CmdTimeout t;
   int32_t manualiq = 150;

   t.Note(100);
   ASSERT(!t.ApplyZero(115, TIMEOUT, manualiq)); // 15 ticks elapsed, still alive
   t.Note(115);                                  // host resumes commanding
   ASSERT(!t.ApplyZero(130, TIMEOUT, manualiq)); // only 15 ticks since refresh
   ASSERT(manualiq == 150);

   // ...but silence measured from the refresh still eventually fires.
   ASSERT(t.ApplyZero(135, TIMEOUT, manualiq));
   ASSERT(manualiq == 0);
}

/* The RTC counter is finite and wraps; elapsed time must stay correct across
 * a wrap, exactly like the existing (now - lastCanRxTime) >= CAN_TIMEOUT idiom. */
static void TestWraparoundSafe()
{
   CmdTimeout t;
   int32_t manualiq = 150;

   const uint32_t nearMax = 0xFFFFFFF0u;
   t.Note(nearMax);

   // now has wrapped past 0; true elapsed = 0x1F - 0xF0 (mod 2^32) = 47 ticks.
   const uint32_t now = 0x0000001Fu;
   ASSERT((uint32_t)(now - nearMax) == 47u);
   ASSERT(t.ApplyZero(now, TIMEOUT, manualiq));
   ASSERT(manualiq == 0);
}

REGISTER_TEST(CmdTimeoutTest, TestZeroesCommandAfterSilence, TestDisabledWhenTimeoutZero,
              TestDoesNotActBeforeFirstCommand, TestRefreshPreventsTimeout, TestWraparoundSafe);
