# Encoder (inc_encoder.cpp)

## Description
`Encoder` (`include/inc_encoder.h`, `src/inc_encoder.cpp`) is a static class
that turns raw rotor-position hardware into a normalized rotor angle and
frequency for both PWM build variants.

Modes (`Encoder::mode`, selected by `encmode`, `ENCMODES` = "0=Single, 1=AB,
2=ABZ, 3=SPI, 4=Resolver, 5=SinCos"): `SINGLE` (one pulse channel, angle
reconstructed by pulse counting + interpolation), `AB`/`ABZ` (quadrature,
hardware encoder-mode counter; `ABZ` additionally waits for a north-marker
EXTI edge), `SPI` (bit-banged AD2S-family absolute encoder), `RESOLVER`/
`SINCOS` (analog sin/cos transducer decoded via `SineCore::Atan2`).

Public interface: `Reset`, `SetMode`, `SeenNorthSignal`, `UpdateRotorAngle`,
`UpdateRotorFrequency`, `SetPwmFrequency`, `GetRotorAngle`, `GetSpeed`,
`GetFullTurns`, `GetRotorFrequency`, `GetRotorDirection`,
`SetImpulsesPerTurn`, `SwapSinCos`, `SetSinCosOffset`, `ResetDistance`,
`GetDistance`.

`param_prj.h` params: `encmode`, `numimp` (pulses/rev, `SINGLE`/`AB`/`ABZ`),
`respolepairs` (resolver/sin-cos electrical pole pairs; also gates
`fullTurns` counting in every mode), `sincosofs`. Used identically by both
`CONTROL=SINE` and `CONTROL=FOC` builds.

## Why?
The five position-feedback hardware types need different timer/ADC/DMA
setups and decode math, but downstream code (PWM generation, throttle/regen,
direction logic) only needs angle, frequency, and direction. One static
class with a common output shape keeps the rest of the firmware
mode-agnostic.

## Drawbacks
- All state is static/global — one encoder instance, no way to unit-test
  the hardware-facing modes without real timer/ADC/DMA peripherals. Only
  the pure wraparound arithmetic in `UpdateTurns` is host-testable, and
  isn't currently covered by a dedicated host test.
- `SINGLE` mode cannot measure direction — `detectedDirection` is just the
  passed-in `dir` (i.e. `seldir`), not sensed (F23, `FORK_NOTES.md`). Any
  downstream guard comparing `GetRotorDirection() != seldir` is inert in
  this mode.
- Below ~2.78 Hz electrical (`STABLE_ANGLE`, resolver/SPI/sin-cos modes)
  frequency is forced to exactly 0 rather than extrapolated — real-but-slow
  rotation and "stopped" are indistinguishable to callers.
  `RegenTaperHold` (`doc/regentaperhold.md`) papers over the consequences
  at the throttle layer; the ambiguity in `Encoder` itself is unfixed.
- `SINGLE`-mode angle interpolation is a linear guess between real pulse
  samples, clamped to not overshoot the next expected pulse (F22) but still
  an extrapolation, not a measurement.

## Architecture
`UpdateRotorAngle(dir)` runs from the PWM ISR (`PwmGeneration::Run`, every
cycle) and updates the mode-specific `angle` (`uint16_t`, 65536/revolution).
For `AB`/`ABZ` it reads the hardware encoder-mode timer counter directly;
for `SPI`/`RESOLVER`/`SINCOS` it calls the mode's `GetAngle*()` decoder then
`UpdateTurns()` (signed delta with 180°-wrap correction, accumulated into
`turnsSinceLastSample`).

`UpdateRotorFrequency(callingFrequency)` runs from the 100 Hz scheduler task
(`Ms10Task`, a separate hardware timer from the PWM timer) and drains
`turnsSinceLastSample` into a frequency estimate: always computed for
`AB`/`ABZ`; for `RESOLVER`/`SPI`/`SINCOS` only once `startupDelay` has
elapsed and accumulated angle exceeds `STABLE_ANGLE`, else forced to 0.
`SINGLE` mode computes frequency inline in `UpdateRotorAngle` from the last
measured pulse period instead.

**Race fix (F13/T6, upstream-tracked)**: `turnsSinceLastSample` is written
from the higher-priority PWM ISR and read-and-cleared from the
lower-priority scheduler ISR — a plain read-then-clear could lose an
increment landing between the two statements. The fix copies the value out
and subtracts only what was read, inside
`cm_disable_interrupts()`/`cm_enable_interrupts()`, preserving a concurrent
increment for the next call. See `FORK_NOTES.md` F13.

**Angle-overshoot clamp (F22, upstream-tracked)**: in `SINGLE` mode,
`interpolatedAngle` is clamped to `anglePerPulse` before accumulation.
Without it, decelerating past a pulse boundary could extrapolate beyond the
next real pulse then snap back on arrival — a sawtooth in synthesized
angle. See `FORK_NOTES.md` F22.

**Direction-guard documentation (F23)**: the `SINGLE`-mode
`detectedDirection = dir` assignment is unchanged but now carries a comment
naming the downstream guard it silently disables. No behavior change; see
`FORK_NOTES.md` F23.

Resolver/sin-cos decode (`GetAngleResolver`, `GetAngleSinCos`,
`DecodeAngle`) drives a square-wave exciter through a 3-pole analog filter
and samples returned sin/cos via injected ADC on a hardware-timed trigger;
angle comes from `SineCore::Atan2`. `resolverMin`/`resolverMax` track
amplitude to detect a not-yet-settled or disconnected resolver
(`ERR_LORESAMP`).

Fixed-point convention: `angle` is `uint16_t`, 65536/revolution, shared with
`PwmGeneration`. `GetRotorFrequency()` returns `u32fp` (`libopeninv`).

## Stability
No dedicated host test exists for this file. The timer/ADC/DMA-driving
modes require real hardware and have not been bench-verified on this fork.
The F13 race fix and F22 clamp are logic changes inside ISR-driven code with
no hardware pass yet — treat both as unvalidated until bench-tested.
