# QClamp (qclamp.h)

## Description
`QClamp` (`include/qclamp.h`) is a small policy class, added in this fork,
that computes hysteresis- and slew-limited bounds for the FOC build's
q-axis current-controller output. Used exclusively by
`src/pwmgeneration-foc.cpp`; the SINE build doesn't use it.

Public interface: `Update(s32fp frqFiltered, int32_t qlimit, int dir, s32fp
thresholdHz)` (advance clamp state by one control-loop cycle), `GetMinLim()
const`, `GetMaxLim() const`.

Call site (`PwmGeneration::Run`, FOC build):
```cpp
qClamp.Update(frqFiltered, qlimit, dir, FP_FROMINT(Param::GetInt(Param::qlimfrq)));
qController.SetMinMaxY(qClamp.GetMinLim(), qClamp.GetMaxLim());
```

`param_prj.h` param: `qlimfrq` (id 165, Hz, range 0-100, default 30). `0`
disables the restriction entirely (bounds always walk to `±qlimit`) — the
value this fork's target dyno hardware runs with. Any nonzero value
reproduces the restricted-quadrant behavior below, gated by that frequency,
matching stock's threshold-driven restriction.

## Why?
Below a low-speed threshold, the resolver/encoder angle estimate is
unreliable enough that firmware restricts q-axis current to one polarity
(matching commanded direction) rather than allowing full regenerative
braking — upstream behavior, unchanged here. The bug (F1, `FORK_NOTES.md`)
was in *how* the restriction applied: a bare `if (frqFiltered <
QLIMIT_FREQUENCY)` compare switched the allowed range instantly and
chattered whenever `frqFiltered` sat near the threshold with any noise,
spiking low-speed current each flip. `QClamp` adds a hysteresis band and
slews the returned bounds toward target instead of snapping, while
preserving the original policy's instant response to a *shrinking*
`qlimit` (the voltage-circle limit, unrelated to the low-speed
restriction).

Extracted as a dependency-free class (no `Param::` reads, no
`PiController` calls) so the hysteresis/slew logic is host-testable in
isolation — `test/test_qclamp.cpp` exercises it directly, impractical
against the full FOC loop.

## Drawbacks
- Hysteresis band (`±2 Hz`) and slew rate (`qlimit / 256` per call, full
  transition ≈ 256 ISR cycles ≈ 29 ms at 8789 Hz) are fixed constants, not
  parameters — tuning either needs a rebuild.
- `Update()` must be called every control-loop cycle for the documented
  slew timing to hold; a different call rate silently changes the
  effective transition time with no check.
- Single static instance (`qClamp` in `pwmgeneration-foc.cpp`) implicitly
  assumes single-threaded, single-motor use.
- No test exercises a live `qlimit`-shrink concurrent with an in-progress
  slew transition (both tested independently per `test_qclamp.cpp`).

## Architecture
`Update()` first evaluates hysteresis on `frqFiltered` against
`thresholdHz`: `thresholdHz == 0` always clears `restricted`; otherwise
enters "restricted" below `thresholdHz - 2 Hz`, leaves it above
`thresholdHz + 2 Hz`, no transition in the deadband (state carries over).

Target bounds: unrestricted is always `[-qlimit, qlimit]`; restricted opens
only the half matching `dir` (`dir <= 0` → `[-qlimit, 0]`, `dir >= 0` →
`[0, qlimit]`). `StepTowards()` walks current `qMinLim`/`qMaxLim` toward
targets by at most `qlimit / 256` per call, per bound independently.

Both bounds are then hard-clamped to the *live* `±qlimit` every call
(`qMinLim = MAX(qMinLim, -qlimit); qMaxLim = MIN(qMaxLim, qlimit)`) —
reproducing stock's `SetMinMaxY(-qlimit, qlimit)` recomputed every cycle
from a possibly-shrinking `qlimit` (from `FOC::GetQLimit`), so a shrinking
`qlimit` still takes effect instantly. Only the restricted/unrestricted
transition and range-opening are slewed; the range never sits wider than
the current voltage-circle constraint.

Fixed-point convention: `frqFiltered`/`thresholdHz` are `s32fp` Q5
(`my_fp.h`, `libopeninv`); `qlimit`/`qMinLim`/`qMaxLim`/slew step are plain
`int32_t` modulation digits (matching `FOC::GetMaximumModulationIndex()`'s
scale, e.g. `modmax` default 37836).

## Stability
Host-tested (`test/test_qclamp.cpp`) against fixed threshold/qlimit
scenarios and compile-checked in the FOC build. No hardware bring-up on
this fork — hysteresis band and slew rate are unvalidated against a real
motor's low-speed behavior.
