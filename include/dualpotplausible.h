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
#ifndef DUALPOTPLAUSIBLE_H
#define DUALPOTPLAUSIBLE_H

/* T25 [FORK]: dual-pot cross-channel plausibility check.
 *
 * Successor to upstream's deleted Throttle::CheckDualThrottle. The current
 * DUALCHANNEL path (VehicleControl::GetUserThrottleCommand) range-checks each
 * channel independently then MIN-selects the command, with NO cross-channel
 * agreement test — so two channels that are BOTH individually in range but
 * disagree (one stuck at a plausible value) are silently reconciled to the
 * lower reading. Jotham's ruling 2026-07-23 (option b): keep the MIN-select
 * for the command (unchanged), but ADDITIONALLY raise a throttle-error flag so
 * a stuck-but-in-range pot is never silent.
 *
 * Upstream's CheckDualThrottle used a hardcoded 10-percentage-point tolerance
 * on the normalized 0..100% channel readings (ABS(diff) > 10 → fault) and, on
 * failure, forced the command to potmin[0] (i.e. zeroed it). We keep the same
 * tolerance semantics (diff on the normalized percent scale, strict >) but make
 * the tolerance a runtime param and DO NOT alter the command — the MIN-select
 * still owns the setpoint; this only decides whether to raise the fault.
 *
 * Off encoding: tolerance == 0 disables the check (returns false / plausible),
 * so the shipped default (param default 0) is behaviourally a no-op.
 *
 * Kept pure and param/MMIO-agnostic (cmdtimeout.h / manualclamp.h precedent) so
 * the host suite drives the real decision logic. */
class DualPotPlausibility
{
   public:
      /** True iff the two normalized channel percentages disagree by MORE than
       * `tolerance` percentage points. A tolerance of 0 disables the check and
       * always returns false (plausible), giving the default-off no-op. The
       * comparison is strict (> tolerance), mirroring upstream's `diff > 10`. */
      static bool Implausible(float potnom1, float potnom2, float tolerance)
      {
         if (tolerance <= 0) return false;
         float diff = potnom1 - potnom2;
         if (diff < 0) diff = -diff;
         return diff > tolerance;
      }
};

#endif // DUALPOTPLAUSIBLE_H
