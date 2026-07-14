# RegenTaperHold (regentaperhold.h)

## Description
`RegenTaperHold` (`include/regentaperhold.h`) is a small policy class,
added in this fork, that holds the last-known regen taper factor across the
encoder's zero-frequency deadband instead of letting regen torque snap to
zero at that boundary. Used by `src/vehiclecontrol.cpp`
(`VehicleControl::ProcessThrottle()`), which runs identically in both
`CONTROL=SINE` and `CONTROL=FOC` builds.

Public interface: `Apply(float rotorfreq, float brkrampstr, float
finalSpnt)` — advance hold state by one call, return the (possibly
tapered/held) throttle command.

Call site (`ProcessThrottle`, 100 Hz):
```cpp
float rotorfreq = FP_TOFLOAT(Encoder::GetRotorFrequency());
float brkrampstr = Param::GetFloat(Param::regenrampstr);
finalSpnt = regenTaperHold.Apply(rotorfreq, brkrampstr, finalSpnt);
```

`param_prj.h` param consumed via the call site (not read internally):
`regenrampstr` — frequency below which regen commands taper toward zero
rather than applying at full magnitude. `RegenTaperHold` reads no `Param::`
values itself; it's a pure function of its three arguments plus state.

## Why?
`Encoder::GetRotorFrequency()` reports exactly `0` below the encoder's
internal deadband (`STABLE_ANGLE`, ~2.78 Hz electrical on resolver/SPI/
sin-cos — see `doc/inc_encoder.md`), meaning "below the deadband," not
"stopped." The original taper multiplied any regen command by
`rotorfreq / brkrampstr` whenever `rotorfreq < brkrampstr`. Because
`rotorfreq` reads exactly `0` at low speed, that snapped regen torque
straight to zero at the deadband boundary, and could flicker if true speed
hovered near it (F4, `FORK_NOTES.md`).

`RegenTaperHold` reproduces the original policy unchanged (`finalSpnt`
passes through untouched unless it's regen below `brkrampstr`) but, when
`rotorfreq` reads exactly `0` inside that window, holds the last nonzero
taper factor for up to 500 ms before releasing to zero, instead of snapping
immediately. A recovering nonzero reading immediately resumes normal taper
and refreshes both the held factor and the hold window.

Extracted as a dependency-free class (no `Param::` reads) for the same
reason as `QClamp` (`doc/qclamp.md`) — host-testable via
`test/test_regentaperhold.cpp`.

## Drawbacks
- The 500 ms hold (`HOLD_CALLS = 50` at the 100 Hz `ProcessThrottle` rate)
  is a fixed constant, not a parameter; assumes `Apply()` is called at
  exactly 100 Hz — a different rate silently changes the effective hold
  duration.
- Doesn't distinguish "genuinely stopped" from "below the encoder's
  deadband" — papers over `Encoder::GetRotorFrequency()`'s zero-reading
  ambiguity rather than resolving it. A truly stopped motor still gets
  regen torque at the last held factor for up to 500 ms — a deliberate
  smooth-taper-vs-brief-continued-torque trade-off, not a correctness fix.
- Single static instance (`regenTaperHold` in `vehiclecontrol.cpp`)
  implicitly assumes single-threaded, single-motor use.
- Depends on the caller passing an unsigned `rotorfreq` magnitude and a
  `finalSpnt` sign convention where negative means regen — get either
  wrong at a new call site and the taper condition (`finalSpnt >= 0 ||
  rotorfreq >= brkrampstr`) silently no-ops.

## Architecture
`Apply()` first checks whether the call is in the taper window: any
non-regen command (`finalSpnt >= 0`) or speed at/above `brkrampstr` resets
`holdCounter` to 0 and returns `finalSpnt` unchanged — matching the
original condition, so behavior outside the deadband edge case is
unchanged.

Inside the window, three cases: (1) `rotorfreq != 0` — compute the taper
factor normally (`rotorfreq / brkrampstr`), refresh the hold
(`holdCounter = HOLD_CALLS`); (2) `rotorfreq == 0` and `holdCounter > 0` —
decrement the counter, return the *previous* `heldFactor * finalSpnt`; (3)
`rotorfreq == 0` and `holdCounter == 0` (expired) — release `heldFactor` to
0, so later calls return 0 until a real nonzero reading resumes case 1.

Uses plain `float` throughout (not `s32fp`), matching `VehicleControl`'s
throttle-processing convention — unlike the fixed-point PWM-ISR-level code
in `PwmGeneration`.

## Stability
Host-tested (`test/test_regentaperhold.cpp`) against hold/release
transitions and compile-checked in both `CONTROL=SINE`/`CONTROL=FOC`
builds. No hardware bring-up on this fork — the 500 ms hold window's
real-world effect on regen feel/safety is unvalidated.
