# VehicleControl

## Description

`src/vehiclecontrol.cpp` / `include/vehiclecontrol.h`. A static (no-instance)
class that turns raw inputs — pots, digital inputs, the vehicle CAN control
frame, temperature sensors, DC bus voltage — into the values `stm32_sine.cpp`'s
`Ms10Task`/`Ms100Task` act on: a final throttle/torque percentage, UDC, contactor
state, and derived status flags. Built into both SINE and FOC variants;
`#if CONTROL == CTRL_FOC` gates the field-weakening derate block inside
`ProcessThrottle()`, and a `hwRev != HW_TESLA` / `hwRev == HW_ZOE` /
`hwRev == HW_PRIUS` / `hwRev == HW_BMWI3HS` branch pattern runs throughout for
per-board differences.

Public interface (all static):
- `SetCan(CanHardware*)` — registers the vehicle control-frame CAN callback.
- `ProcessUdc()` — reads/filters DC bus voltage, drives overvoltage and
  precharge-timeout faults, returns the filtered value.
- `ProcessThrottle()` — the throttle/cruise/idle/hill-hold/regen pipeline;
  returns the final torque-percent setpoint.
- `SelectDirection()`, `CruiseControl()`, `GetDigInputs()`, `CalcAndOutputTemp()`,
  `SetContactorsOffState()`, `PostErrorIfRunning()`.

Parameters consumed (non-exhaustive): direction/cruise — `seldir`, `dirmode`,
`dirchrpm`, `cruisemode`, `cruisespeed`, `din_cruise`; throttle path — `fstat`,
`fmax`, `throtramp`, `throtramprpm`, `regenrampstr`, `brklightout`, `potmode`,
`regenpreset`, `pot`/`pot2`; UDC/precharge — `udcmin`, `udcmax`, `udclim`,
`udcgain`, `udcsw`, `udcofs`, `snshs`; temperature — `tmphs`/`tmphsmax`,
`tmpm`/`tmpmmax`, `pwmgain`, `pwmofs`, `pwmfunc`, `fanthresh`; CAN control
frame — `controlid`, `controlcheck`; digital inputs — `canio` and the `din_*`
set. `Param::Change()`'s `default:` branch (`doc/stm32_sine.md`) loads most of
these into `Throttle::` static fields; `VehicleControl` reads the rest live.

## Why?

This is the vehicle-integration layer: everything hardware- or vehicle-specific
that isn't the motor control loop itself lives here, so `pwmgeneration*.cpp`
only ever sees a torque percentage and doesn't need to know about brake pedals,
cruise switches, or CAN-sourced pot values. Consolidating the CAN
control-frame parsing, direction selection, and contactor sequencing in one
place keeps the safety-relevant interlocks (`STAT_*` in `Ms10Task`) auditable
from a small number of call sites.

## Drawbacks

- `ProcessThrottle()` is a long function doing several unrelated things
  (ramp, BMS limit, UDC/IDC/frequency/acceleration derates, temperature
  derate, brake-light hysteresis, regen taper, direction-sign application) in
  sequence with shared mutable state (`finalSpnt`); the ordering matters and
  is not documented anywhere but the code itself. An inline comment flags a
  known wart, not fixed in this fork: "inconsistency here: in slip control
  negative always means regen" — SINE and FOC disagree on how direction is
  applied to a negative torque command.
- `GetTemps()` special-cases four sensor topologies (Tesla mux'd ADC, Prius
  NTC divider with a documented pull-down-resistor caveat, BMW i3
  SPI-multiplexed ADC, generic lookup) inline; a fifth would grow this
  function further rather than dispatch through a table.
- The CAN control-frame CRC/sequence-counter error handling
  (`CanReceive()`) tolerates up to 5 consecutive errors before it stops
  recovering and requires an inverter restart — that threshold is a magic
  number (`maxErrors = 5`) local to the function, not a parameter.
- `BmwAdcAcquire()` bit-bangs a 4-channel round-robin over SPI on every call
  to `ProcessUdc()`, one channel per 10 ms tick — a full 4-channel update
  takes 40 ms.

## Architecture

`ProcessUdc()` and `ProcessThrottle()` are both called once per `Ms10Task` cycle
(10 ms / 100 Hz, see `doc/stm32_sine.md`); `SelectDirection()` and
`CruiseControl()` run once per `Ms100Task` cycle (100 ms).

Throttle pipeline (`ProcessThrottle()`), in order:
1. Set `Throttle::throttleRamp` based on whether rotor speed is below
   `throtramprpm`.
2. Read the user throttle command (`GetUserThrottleCommand()`, private) unless
   cruise mode is `CRUISE_POT` (in which case cruise supplies the setpoint).
3. `GetCruiseCreepCommand()` (private) applies idle-speed hold, cruise-speed
   regulation, or hill-hold, and reports whether direction should be
   auto-determined from the result.
4. `Throttle::RampThrottle()` applies the ramp/regen-ramp rate limit.
5. BMS, UDC, IDC, frequency, and acceleration limiters run in sequence
   (`Throttle::BmsLimitCommand` etc. — see `doc/throttle.md`), each narrowing
   `finalSpnt` toward zero.
6. Two temperature derates (heatsink, motor) run and post `ERR_TMPHSMAX` /
   `ERR_TMPMMAX` if active.
7. Brake-light output is derived from `finalSpnt` crossing `brklightout` with a
   2 %-point hysteresis band, clamped to never turn off above zero request.
8. If direction is auto-determined: apply the regen taper/hold across the
   near-zero-speed deadband (`RegenTaperHold`, fork addition — see below),
   apply direction sign (FOC: `Encoder::GetRotorDirection()` for regen,
   `seldir` otherwise; SINE: implicit in the sign of `finalSpnt`), and in FOC
   builds run a second, independent frequency-limit instance
   (`FrequencyLimitCommandFw`) to derate field-weakening current above 110% of
   `fmax`.
9. Force `finalSpnt = 0` when `seldir == 0` (neutral).

`GetUserThrottleCommand()` (private) resolves CAN vs. analog pot source, range
per-channel via `Throttle::CheckAndLimitRange`, dual-channel agreement/fallback
logic, bidirectional-pot handling (`CalcThrottleBiDir`), and finally
`Throttle::CalcThrottle()` for the standard single/dual-pot case.

`ProcessUdc()` filters the raw ADC (or BMW i3 SPI ADC, or nothing on `HW_ZOE`)
through a 2-tap IIR, applies gain/offset, posts `ERR_OVERVOLTAGE` and opens the
contactor above `udclim`, posts `ERR_PRECHARGE` if precharge hasn't reached
`udcsw/2` within `PRECHARGE_TIMEOUT` (5 s), and — SINE only — recomputes
field-weakening frequency and boost from the bus-voltage deviation from
`udcnom`.

`CanReceive()` is registered as a `FunctionPointerCallback` against the CAN
control frame ID (`controlid`); it validates an optional CRC
(`controlcheck`) and a 2-bit rolling sequence counter duplicated in both CAN
words, tolerating up to 5 consecutive bad frames before it stops recovering
and forces pot/cruise values to a safe (0) state. Good frames populate `pot`,
`pot2`, `canio`, and (depending on `cruisemode`) `cruisespeed`.

### Fork additions

Three changes in this file are fork-added (see FORK_NOTES.md for the full
finding writeups):
- **Regen taper hold (F4/T6).** `Encoder::UpdateRotorFrequency()` reports
  exactly `0` below its encoder deadband, which is "below the deadband," not
  "stopped." The original code multiplied the regen taper factor
  (`rotorfreq / brkrampstr`) straight through, so torque snapped to zero (and
  could flicker) right at that boundary. `ProcessThrottle()` now delegates to
  a static `RegenTaperHold` instance (`include/regentaperhold.h`) that holds
  the last nonzero taper factor for up to 500 ms (50 calls at the 100 Hz rate
  this function runs at) before releasing to zero, and resumes normal taper
  immediately on any nonzero frequency reading. It is a dependency-free policy
  object (no `Param::` reads) specifically so it's host-testable in isolation
  — see `test/test_regentaperhold.cpp`.
- **`FrequencyLimitCommandFw` split (F16/T12).** The field-weakening derate call
  now uses its own `Throttle` IIR filter state instead of sharing the one used
  by the main torque-command frequency limiter (see `doc/throttle.md`).
- **`bmwAdcNextChan` off-by-one (F16/T12).** The round-robin channel index
  wrapped on `== maxChan` instead of `>= maxChan - 1`, which meant it briefly
  addressed one past the last valid index before wrapping. Fixed to
  `>= maxChan - 1`.

## Stability

Covered by host tests for the CAN control-frame path (CRC pass/fail, sequence
error recovery, brake-light hysteresis — `test/test_vcu.cpp`) and compiled for
both SINE and FOC. Not hardware-validated in this fork.
