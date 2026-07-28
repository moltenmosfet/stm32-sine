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
#ifndef THROTCLAMP_H
#define THROTCLAMP_H

/* G8b [FORK]: firmware-side magnitude clamp on the THROTTLE current path
 * (FMEA G8 follow-up; the scope G8 explicitly excluded, defends ABS-1 / RTL-2).
 *
 * G8 capped the CAN manualiq/manualid path and scoped its claim there,
 * recording the throttle path as an open host-reachable uncapped current
 * command. This is that gap: in the FOC build the throttle percent becomes a
 * stator current request as
 *
 *     is = throtcur [A/%] * torquePercent [%]
 *
 * and `throtcur` is a persisted param settable over the SAME CAN/SDO path as
 * everything else. Its param range is 0..10 A/%, and the throttle percent is
 * bounded to +/-100, so the pre-G8b firmware bound on the commanded stator
 * current magnitude is 1000 A -- i.e. 2.5x the manual path's +/-400 A range
 * that G8 thought worth capping. A host that writes throtcur high (or a
 * commissioning slip) reaches the current controllers at that magnitude.
 *
 * The clamp is applied to `is` BEFORE FOC::Mtpa splits it into id/iq. That is
 * the correct seam: Mtpa is magnitude-preserving (idref^2 + iqref^2 == is^2 by
 * construction, libopeninv/src/foc.cpp), so bounding |is| bounds the actual
 * commanded stator current magnitude. Clamping id/iq separately after the split
 * would NOT bound the vector magnitude, and clamping the throttle PERCENT would
 * leave throtcur free to scale past the ceiling. Symmetric by design: `maxMag`
 * is a magnitude, so it bounds drive and regen identically.
 *
 * SCOPE: the throttle->current conversion only. The manualiq/manualid path
 * (G8), the regen ramps and taper, and every derate/limit in ProcessThrottle
 * are untouched -- they all act on the throttle PERCENT upstream of this seam
 * and keep working exactly as before, they just can no longer produce a current
 * request above the ceiling.
 *
 * FOC-ONLY, honestly: `throtcur` lives in THROTTLE_PARAMETERS_FOC and is
 * referenced only from pwmgeneration-foc.cpp. The SINE build's
 * SetTorquePercent produces an amplitude/slip (voltage) command, not a current
 * command, so there is no throttle->current seam to clamp there. Same scoping
 * as G8, for the same structural reason.
 *
 * Kept pure and param/MMIO-agnostic (cmdtimeout.h / manualclamp.h /
 * dualpotplausible.h precedent) so the host suite drives the real clamp code
 * rather than a copy; the Param glue in pwmgeneration-foc is firmware-compile-
 * checked. Unlike G8's clamp this one operates on `float`, not s32fp: the
 * throttle current path is float end to end (Param::GetFloat -> FOC::Mtpa), and
 * converting to fixed point purely to reuse G8's helper would add a rounding
 * step in the torque path for no benefit. The arithmetic is deliberately the
 * same shape as manualclamp.h's -- see FORK_NOTES G8b for why the two were left
 * as separate typed helpers rather than merged into one template.
 *
 * BOOT LATCH (defends the ceiling against the CAN channel it distrusts): the
 * `throtcurmax` param is settable over the same CAN/SDO path as `throtcur`
 * itself, so a host bug could simply raise the ceiling and then command past
 * it. EffectiveThrottleCeiling closes that by taking the MIN of the live param
 * and a value latched once at init (after flash load). A downward runtime
 * change takes effect immediately (MIN picks the lower live value); a runtime
 * attempt to RAISE the ceiling has no live effect -- it only takes hold after
 * set -> flash `save` -> power cycle re-latches at the new value, a deliberate
 * commissioning act, not a runtime write. No write-path rejection is needed;
 * the guard is pure read-side arithmetic, so the host suite drives it with the
 * latch injected. */
static inline float ClampThrottleCurrent(float cmd, float maxMag)
{
   if (maxMag < 0) maxMag = 0;      // param min pins this at >=0; defensive only
   if (cmd >  maxMag) return  maxMag;
   if (cmd < -maxMag) return -maxMag;
   return cmd;
}

/* Effective authority ceiling = MIN(live param, boot-latched value). See the
 * BOOT LATCH note above. Both inputs are non-negative (PARAM_ENTRY min=0);
 * ClampThrottleCurrent applies its own defensive negative guard downstream. */
static inline float EffectiveThrottleCeiling(float liveMax, float latchedMax)
{
   return liveMax < latchedMax ? liveMax : latchedMax;
}

#endif // THROTCLAMP_H
