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
#ifndef QCLAMP_H_INCLUDED
#define QCLAMP_H_INCLUDED

#include "my_fp.h"

/** Hysteresis + slew-limited bounds for the low-speed q-axis clamp (F1).
 * Below thresholdHz the q-axis PI output is restricted to one quadrant
 * (regen blocked) because the resolver reading is unreliable near
 * standstill; above it, the full +-qlimit range is open. A binary frequency
 * compare chatters open/closed on noise right at the threshold and the
 * clamp used to switch instantly, both of which show up as low-speed
 * current spikes. This class enters/leaves the restricted region with a
 * +-2 Hz hysteresis band and walks the returned bounds toward their target
 * a step at a time instead of snapping them.
 *
 * Kept as a small, dependency-free policy object (no Param:: reads, no
 * PiController calls) so it is host-testable on its own; the caller feeds
 * it the current inputs each cycle and applies GetMinLim()/GetMaxLim() to
 * whatever it is clamping.
 */
class QClamp
{
   public:
      QClamp() : restricted(true), qMinLim(0), qMaxLim(0) {}

      /** Advance the clamp state by one ISR cycle.
       * \param frqFiltered filtered rotor frequency, s32fp Q5
       * \param qlimit q-axis voltage magnitude limit (modulation digits), >= 0
       * \param dir direction sign (-1, 0, 1); while restricted, selects which
       *        half of the range is open (forward: [0,qlimit], reverse: [-qlimit,0])
       * \param thresholdHz hysteresis center frequency, s32fp Q5; 0 disables
       *        the restriction entirely (bounds always walk to +-qlimit)
       */
      void Update(s32fp frqFiltered, int32_t qlimit, int dir, s32fp thresholdHz)
      {
         if (0 == thresholdHz)
         {
            restricted = false;
         }
         else if (restricted && frqFiltered > thresholdHz + FP_FROMINT(2))
         {
            restricted = false;
         }
         else if (!restricted && frqFiltered < thresholdHz - FP_FROMINT(2))
         {
            restricted = true;
         }

         int32_t targetMin = restricted ? (dir <= 0 ? -qlimit : 0) : -qlimit;
         int32_t targetMax = restricted ? (dir >= 0 ? qlimit : 0) : qlimit;
         int32_t maxStep = qlimit / slewDivisor;

         qMinLim = StepTowards(qMinLim, targetMin, maxStep);
         qMaxLim = StepTowards(qMaxLim, targetMax, maxStep);
      }

      int32_t GetMinLim() const { return qMinLim; }
      int32_t GetMaxLim() const { return qMaxLim; }

   private:
      static const int32_t slewDivisor = 256; //full transition ~= 256 ISR cycles (~29 ms at 8789 Hz)

      static int32_t StepTowards(int32_t current, int32_t target, int32_t maxStep)
      {
         int32_t diff = target - current;

         if (diff > maxStep)
            diff = maxStep;
         else if (diff < -maxStep)
            diff = -maxStep;

         return current + diff;
      }

      bool restricted;
      int32_t qMinLim, qMaxLim;
};

#endif // QCLAMP_H_INCLUDED
