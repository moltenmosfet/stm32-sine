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
#ifndef MANUALCLAMP_H
#define MANUALCLAMP_H
#include <stdint.h>

/* G8 [FORK]: firmware-side magnitude clamp on the CAN manualiq/manualid torque
 * path (FMEA G8, defends ABS-1 / RTL-2).
 *
 * The dyno's absorber is driven entirely through the manualiq/manualid current
 * setpoints written over CAN by the CM4 virtual-inertia loop. The only firmware
 * bound today is the ±400 A param range; a divergent or garbage-but-in-range
 * host command therefore reaches the current controllers at full authority. G8
 * is the defence-in-depth ceiling the FMEA requires OUTSIDE that failure
 * boundary: a fork-settable cap (`manualiqmax`) that no host bug can exceed,
 * because it is applied in the firmware at the summing junction, not on the CM4.
 *
 * Kept pure and param/MMIO-agnostic (cmdtimeout / RegenTaperHold / CheckOverrun
 * precedent) so the host suite drives the real clamp code rather than a copy;
 * the Param glue in pwmgeneration-foc is firmware-compile-checked. The clamp is
 * monotone in the raw stored representation, so it is applied directly on the
 * s32fp (int32_t) fixed-point value with no scale conversion. Symmetric by
 * design: `maxMag` is a magnitude and bounds both command polarities.
 *
 * BOOT LATCH (defends the ceiling against the CAN channel it distrusts): the
 * `manualiqmax` param is settable over the same CAN/SDO path as the torque
 * command it caps, so a host bug could simply raise the ceiling and then
 * command past it. EffectiveManualCeiling closes that by taking the MIN of the
 * live param and a value latched once at init (after flash load). A downward
 * runtime change takes effect immediately (MIN picks the lower live value); a
 * runtime attempt to RAISE the ceiling has no live effect — it only takes hold
 * after set -> flash `save` -> power cycle re-latches at the new value, a
 * deliberate commissioning act, not a runtime write. No write-path rejection is
 * needed; the guard is pure read-side arithmetic, so the host suite drives it
 * with the latch injected. */
static inline int32_t ClampManualCurrent(int32_t cmd, int32_t maxMag)
{
   if (maxMag < 0) maxMag = 0;      // param min pins this at >=0; defensive only
   if (cmd >  maxMag) return  maxMag;
   if (cmd < -maxMag) return -maxMag;
   return cmd;
}

/* Effective authority ceiling = MIN(live param, boot-latched value). See the
 * BOOT LATCH note above. Both inputs are non-negative (PARAM_ENTRY min=0);
 * ClampManualCurrent applies its own defensive negative guard downstream. */
static inline int32_t EffectiveManualCeiling(int32_t liveMax, int32_t latchedMax)
{
   return liveMax < latchedMax ? liveMax : latchedMax;
}

#endif // MANUALCLAMP_H
