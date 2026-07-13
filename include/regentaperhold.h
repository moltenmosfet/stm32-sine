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
#ifndef REGENTAPERHOLD_H_INCLUDED
#define REGENTAPERHOLD_H_INCLUDED

/** Holds the regen taper factor across the encoder's zero-frequency
 * deadband (F4). Encoder::UpdateRotorFrequency() reports lastFrequency as
 * exactly 0 below its ~2.78 Hz elec STABLE_ANGLE deadband -- that means
 * "below the deadband", not "stopped". Multiplying the regen taper
 * (rotorfreq / brkrampstr) straight through makes the factor snap to zero
 * at the deadband boundary instead of tapering, and flicker 0 <-> nonzero
 * sample to sample right at the edge.
 *
 * This class reproduces the original taper policy (finalSpnt unchanged
 * unless rotorfreq < brkrampstr and finalSpnt < 0) but, when the encoder
 * reports exactly 0 inside that regen window, holds the last nonzero
 * taper factor for up to HOLD_CALLS calls before releasing to zero. A
 * recovering (nonzero) frequency immediately resumes the normal taper and
 * refreshes the held factor and the hold window.
 *
 * Kept as a small, dependency-free policy object (no Param:: reads) so it
 * is host-testable on its own; the call site owns a static instance and
 * calls Apply() once per ProcessThrottle cycle (100 Hz), same pattern as
 * QClamp (include/qclamp.h, T4).
 */
class RegenTaperHold
{
   public:
      RegenTaperHold() : heldFactor(0), holdCounter(0) {}

      /** Advance the hold state by one call and return the (possibly
       * tapered/held) throttle command.
       * \param rotorfreq measured rotor frequency magnitude, Hz; exactly 0
       *        means "below the encoder's deadband", not "stopped"
       * \param brkrampstr regen ramp-start frequency, Hz
       * \param finalSpnt current throttle command; only regen (< 0) commands
       *        below brkrampstr are tapered -- everything else passes
       *        through unchanged, same as the original condition
       */
      float Apply(float rotorfreq, float brkrampstr, float finalSpnt)
      {
         if (finalSpnt >= 0 || rotorfreq >= brkrampstr)
         {
            holdCounter = 0; //not in the taper window; nothing to hold
            return finalSpnt;
         }

         if (rotorfreq != 0)
         {
            heldFactor = rotorfreq / brkrampstr;
            holdCounter = HOLD_CALLS;
         }
         else if (holdCounter > 0)
         {
            holdCounter--;
         }
         else
         {
            heldFactor = 0; //hold expired, release to zero
         }

         return heldFactor * finalSpnt;
      }

   private:
      //500 ms at the 100 Hz rate ProcessThrottle calls this
      static const int HOLD_CALLS = 50;

      float heldFactor;
      int holdCounter;
};

#endif // REGENTAPERHOLD_H_INCLUDED
