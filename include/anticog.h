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
#ifndef ANTICOG_H_INCLUDED
#define ANTICOG_H_INCLUDED

#include <stdint.h>

/** Anti-cogging triangle wave shaping (F5/T11).
 *
 * GenerateAntiCoggingSignal() folds a full electrical revolution (0..65535
 * digits) into a quarter-wave, then scales it up to a +-32767ish triangle.
 * The original code did that final scale as `uint16_t antiCog = 4 * angle`,
 * which wraps 65536->0 right at the 270 degree branch boundary (folded
 * angle 16384, raw angle 49151->49152) and emits a one-cycle full-scale
 * glitch into the d-axis feedforward. Doing the multiply/subtract in
 * int32_t keeps the triangle continuous across every branch boundary.
 *
 * Kept as a small, dependency-free pure function (no Param:: reads) so the
 * triangle shape is host-testable across all 65536 angle values on its
 * own, same pattern as QClamp (include/qclamp.h, T4) and RegenTaperHold
 * (include/regentaperhold.h, T6).
 *
 * Caveat out of scope for this fix: MeasureCoggingCurrent() (the caller
 * that derives the coggingCurrent magnitude fed into this shape) estimates
 * cogging amplitude as the raw min-max spread of id over one electrical
 * revolution -- that measures ALL id disturbance, including this
 * injection's own effect on id. A mis-phased cogph therefore
 * self-reinforces up to the cogmax clamp instead of converging. Fixing the
 * estimator (synchronous demodulation or band-passing) is a separate task.
 */
static inline int32_t AntiCogTriangle(uint16_t angle)
{
   if (angle < 16384) //0 to 90 degrees (65536 digits/rev)
      ; //no change
   else if (angle < 32768) //90 to 180
      angle = 32767 - angle;
   else if (angle < 49152) //180 to 270
      angle = angle - 32767;
   else //270 to 360
      angle = 65535 - angle;

   uint32_t antiCog = 4 * (uint32_t)angle; //32-bit: the folded angle can reach 16384, and 4*16384 == 65536 overflows uint16_t

   return (int32_t)antiCog - 32767;
}

#endif // ANTICOG_H_INCLUDED
