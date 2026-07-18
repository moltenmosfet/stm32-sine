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
#ifndef CMDTIMEOUT_H
#define CMDTIMEOUT_H
#include <stdint.h>

/* T21 [FORK]: manualiq command-silence watchdog.
 *
 * Defence-in-depth BEHIND the hardware GPIO dead-man heartbeat chain (which
 * stays the floor: a hung/crashed host stops pulsing the GPIO line and the
 * hardware chain opens HV regardless of this class). This is the graceful
 * backstop D4 asked for: if the host briefly stalls (GC pause, missed tick)
 * the last torque command latches in firmware; this zeroes it after a bounded
 * silence so the machine de-energizes while the contactor stays closed —
 * recovery needs no re-precharge. It is a command decay, NOT a fault trip:
 * the caller's only action on Expired() is to zero the manualiq value.
 *
 * Kept pure and param/MMIO-agnostic (RegenTaperHold / CheckOverrun precedent)
 * so the host harness drives the real decision logic; the FOC-only Param glue
 * lives in VehicleControl and is compile-checked in firmware. Time is in RTC
 * counter ticks (10 ms/tick), matching the existing CAN_TIMEOUT idiom. */
class CmdTimeout
{
   public:
      /** Record that a manual torque command arrived at time `now`. */
      void Note(uint32_t now) { last = now; seen = true; }

      /** True once a command has been seen at least once, the watchdog is
       * enabled (timeout != 0), and `now` is at least `timeout` ticks past
       * the last command. Unsigned subtraction is wraparound-safe, mirroring
       * the (now - lastCanRxTime) >= CAN_TIMEOUT check in VehicleControl.
       * The seen-gate mirrors canIoActive: the watchdog does nothing until
       * the host has actually started commanding. */
      bool Expired(uint32_t now, uint32_t timeout) const
      {
         return seen && timeout != 0 && (uint32_t)(now - last) >= timeout;
      }

      /** Zero the command reference iff Expired(); return whether it acted.
       * Takes the command by reference and nothing else, so by construction
       * it cannot alter contactor or PWM-enable state — a graceful decay, not
       * a fault trip. */
      bool ApplyZero(uint32_t now, uint32_t timeout, int32_t& iqCmd) const
      {
         if (Expired(now, timeout))
         {
            iqCmd = 0;
            return true;
         }
         return false;
      }

   private:
      uint32_t last = 0;
      bool seen = false;
};

#endif // CMDTIMEOUT_H
