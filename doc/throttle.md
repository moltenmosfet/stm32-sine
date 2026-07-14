# Throttle

## Description

`src/throttle.cpp` / `include/throttle.h`. A static class of pure-ish math
functions plus static state (limiter setpoints, filter state, ramp state) that
implement the throttle curve, cruise/idle-speed control, and the family of
"limit command toward zero if X is out of bounds" derate functions. Built into
both SINE and FOC variants; nothing in this file is `#if CONTROL`-gated. All
callers are in `VehicleControl` (see `doc/vehiclecontrol.md`) and
`stm32_sine.cpp`'s `RunCharger()`.

Public static functions: `CheckAndLimitRange`, `DigitsToPercent`, `CalcThrottle`,
`CalcThrottleBiDir`, `CalcIdleSpeed`, `CalcCruiseSpeed`, `HoldPosition`,
`TemperatureDerate`, `BmsLimitCommand`, `UdcLimitCommand`, `IdcLimitCommand`,
`AccelerationLimitCommand`, `FrequencyLimitCommand`, `FrequencyLimitCommandFw`,
`RampThrottle`, `UpdateDynamicRegenTravel`, `IsThrottlePressed`.

Public static state (set from `Param::Change()`'s default branch in
`stm32_sine.cpp`, not read directly from `Param::` inside this file):
`potmin[2]`/`potmax[2]`, `brknom`, `brknompedal`, `brkmax`, `brkcruise`,
`throtmax`, `throtmin`, `linearity`, `idleSpeed`, `cruiseSpeed`, `speedkp`,
`holdkp`, `speedflt`, `idleThrotLim`, `cruiseThrotLim`, `regenRamp`,
`throttleRamp`, `bmslimhigh`, `bmslimlow`, `accelmax`, `accelflt`, `udcmin`,
`udcmax`, `idcmin`, `idcmax`, `idckp`, `fmax`, `maxregentravelhz`. These map to
`param_prj.h` entries of the same or similar name (e.g. `regentravel` →
`brknom` via `UpdateDynamicRegenTravel`, `offthrotregen` → `brkmax`,
`brakeregen` → `brknompedal`, `cruiseregen` → `brkcruise`).

## Why?

Separating throttle math from the vehicle-integration layer (`VehicleControl`)
keeps the curve/limiter functions free of `Param::`/CAN/digital-IO
dependencies, which is what makes most of them host-testable
(`test/test_throttle.cpp`) without stubbing hardware. Each `*LimitCommand`
function follows the same shape — compute a bound from an error term, clamp it
to the correct sign, then `MIN`/`MAX` it into `finalSpnt` — so a caller can
chain as many of them as needed and each only ever narrows the torque request
toward zero, never widens it.

## Drawbacks

- All state is `static`/global, not instance-based — there is exactly one
  throttle context for the whole firmware. Fine for a single-motor inverter,
  but it means two calls to functions with private filter state (like the old
  single `FrequencyLimitCommand`) silently shared state unless deliberately
  separated (see Architecture below).
- `CalcThrottle()`'s quadratic-blend curve (`quad = potnom² · (1-linearity) +
  potnom · linearity`) and `CalcThrottleBiDir()`'s brknom-deadband-then-rescale
  logic are dense, comment-free arithmetic; understanding the shape of the
  curve requires plotting it, not reading it.
- `AccelerationLimitCommand()` only derates positive (motoring) torque and
  only above `speed > 100`; below that it's a no-op, which is not stated
  anywhere except by reading the condition.
- `RampThrottle()` picks between `RAMPUP`, `RAMPDOWN` at `throttleRamp`, and
  `RAMPDOWN` at `regenRamp` based on the sign/magnitude of `throttleRamped`
  itself (`> 5` threshold) rather than the sign of `potnom` — a torque request
  that's ramping down through the `throttleRamped > 5` boundary switches ramp
  rate mid-ramp.

## Architecture

These functions are called from `VehicleControl::ProcessThrottle()` once per
`Ms10Task` cycle (10 ms / 100 Hz) in a fixed pipeline — see `doc/vehiclecontrol.md`
for the call order. `HoldPosition`/`CalcIdleSpeed`/`CalcCruiseSpeed` are called
from `VehicleControl::GetCruiseCreepCommand()`, also per-cycle.

`RunFrequencyLimit()` (private) is the shared implementation behind
`FrequencyLimitCommand()` and `FrequencyLimitCommandFw()`: it runs a 4-tap IIR
filter on the frequency input, then derates a positive torque request toward
zero as frequency approaches `fmax`. Both public wrappers pass in a distinct
static filter-state reference (`frqFiltered` / `fwFrqFiltered`) so the two
call sites' filters don't interfere with each other.

`UpdateDynamicRegenTravel()` recomputes `brknom` (the "throttle percent below
which is regen" threshold used by `CalcThrottle`) as a function of rotor
frequency when `maxregentravelhz` is nonzero, ramping it from a floor of 3%
up to the configured `regentravel` max as speed increases; at `maxregentravelhz
== 0` it's pinned to `regentravel` unconditionally.

### Fork addition: per-caller frequency-limit filter state (F16/T12)

Originally `FrequencyLimitCommand()` held its IIR filter state in a function-
local `static float frqFiltered`. `VehicleControl::ProcessThrottle()` has two
call sites per cycle that need this behavior — the main torque-command
frequency limiter, and (FOC only) the field-weakening current derate. Sharing
one filter meant the second call each cycle was filtering against a state
value the first call had already advanced, effectively running the filter at
double rate for one of the two signals. The fix (`RunFrequencyLimit`) hoists
the state into two separate `Throttle::` static members (`frqFiltered`,
`fwFrqFiltered`) and gives each caller its own via a
`float& frqFiltered` output parameter. See `test/test_throttle.cpp`'s
`TestFrequencyLimitStateIsolated`.

## Stability

Covered by `test/test_throttle.cpp` (curve linearity, brake-pedal override,
dual-throttle fallback logic, frequency-limit state isolation). Not
hardware-validated in this fork.
