# stm32_sine (firmware entry point)

## Description

`src/stm32_sine.cpp` is the firmware's `main()` and the top-level scheduler wiring
for both build variants: plain `make` builds the SINE (V/f) control firmware,
`CONTROL=FOC make` builds the FOC (field-oriented) firmware. The `CONTROL` macro
(`CTRL_SINE` / `CTRL_FOC`) gates the few sections in this file that differ between
variants (see `#if CONTROL == CTRL_SINE` / `CTRL_FOC` blocks below).

Responsibilities:
- `main()`: clock/RTC/timer/NVIC setup, hardware-variant detection (`io_setup()`),
  parameter load, construction of the CAN stack (`Stm32Can`, `CanMap`, `InverterSdo`),
  registration of the two periodic tasks, terminal construction, and the
  JSON-over-SDO print pump in the idle loop.
- `Ms100Task()` (100 ms period): LED heartbeat, watchdog kick (`iwdg_reset()`),
  CPU-load and uptime/error telemetry, hardware-fault digital-input read on
  REV1/BLUEPILL, brake-pedal-seen latch, `VehicleControl::SelectDirection()` /
  `CruiseControl()`, SINE-only AC voltage estimate, and periodic CAN map send
  when `canperiod == CAN_PERIOD_100MS`.
- `Ms10Task()` (10 ms period): the state machine — reads UDC and throttle via
  `VehicleControl`, evaluates the interlock bits (`STAT_*`) that gate closing the
  DC contactor (emergency stop, motor-protection, throttle-pressed, UDC window,
  brake-pedal-seen), decides `MOD_OFF`/charge/`MOD_RUN` transitions, drives the
  contactor and PWM opmode, and sends the CAN map when `canperiod == CAN_PERIOD_10MS`.
  Comment in the source: "Normal run takes 70µs -> 0.7% cpu load."
- `Param::Change()`: the parameter-write callback invoked by the parameter
  framework (`libopeninv`) whenever a parameter is set (terminal, CAN SDO, or
  boot-time `Param::Change(PARAM_LAST)`). Most branches push a `Param::` value
  into the runtime state of another module (`Throttle::`, `Encoder::`,
  `PwmGeneration::`, `FOC::`). The `default:` branch re-derives nearly all
  throttle/encoder/FOC runtime state from parameters — this is the "full
  reconfiguration" path referenced by FORK_NOTES F14.
- `tim2_isr()` / `tim4_isr()`: hand off to `Stm32Scheduler::Run()`. Which timer
  drives the scheduler depends on hardware variant (`TIM4` on `HW_BLUEPILL`,
  `TIM2` otherwise — see `hwinit.cpp`/`hwdefs.h`).

Consumes parameters broadly (almost all of `param_prj.h` gets touched somewhere
in `Param::Change`'s default branch); the ones referenced directly in this file
are `canspeed`, `outmode`, `manualid`/`manualiq` (FOC only), `throtmax`,
`throtmin`, `idcmin`, `idcmax`, `offthrotregen`, `nodeid`, `opmode`, `chargemode`,
`chargecur`, `tmphs`/`tmphsmax`, `udcsw`, `udclim`, `cruisemode`, `din_*`,
`canperiod`, `tripmode`, `pwmpol`, `bootprec`, `snsm`, `version`, `hwver`,
`regenpreset`.

## Why?

The firmware is organized as two cooperative-scheduled tasks (10 ms, 100 ms)
plus interrupt-driven PWM/current-loop code living in `pwmgeneration*.cpp`. This
file is the seam where "always-on background bookkeeping" (100 ms) is separated
from "the actual run/stop and torque-command state machine" (10 ms) — the state
machine needs a fast, predictable period to keep the contactor-close interlock
and throttle ramp responsive, while telemetry and cruise control tolerate a
slower one. Centralizing all parameter side effects in `Param::Change()` means
any code that changes a parameter — terminal, CAN SDO, or boot — gets identical
downstream behavior without each caller re-deriving runtime state itself.

## Drawbacks

- The `default:` branch of `Param::Change()` recomputes a large amount of state
  on every parameter write that isn't one of the explicitly special-cased ones.
  FORK_NOTES F14 documents this costing tens of microseconds per CAN SDO write,
  which matters for high-rate writes; T7 added the `manualid`/`manualiq`
  fast-path fast-return specifically to avoid paying this cost on every torque
  command frame (fork addition, FOC build only).
  `PwmGeneration::UpdateVoltageLimits()` call after `FOC::SetMaximumModulationIndex()`
  is also a fork addition (re-clamps d/q limits live when `modmax` changes; a
  no-op unless the motor is running).
- `Ms10Task` mixes contactor-safety logic, throttle processing, and CAN
  scheduling in one function; the interlock bits (`STAT_*`) are computed with a
  single bitwise-OR chain that is easy to read wrong at a glance.
- No hardware-in-the-loop test exists for `main()` or the two scheduled tasks —
  they are only exercised indirectly through `VehicleControl`/`Throttle` unit
  tests and compile checks (see `doc/host-tests.md`).

## Architecture

### Module map

```
main()
 ├─ hwinit.cpp        hardware detection, clocks, timers, NVIC, pin mux
 ├─ Stm32Scheduler     (libopeninv) — TIM2/TIM4-driven cooperative scheduler
 │    ├─ Ms100Task (100 ms)
 │    └─ Ms10Task  (10 ms)
 ├─ Stm32Can / CanMap / InverterSdo (CAN hardware, generic param mapping, SDO)
 │    └─ VehicleControl::SetCan()   registers the vehicle CAN control-frame callback
 ├─ VehicleControl      throttle/UDC/temperature/direction/cruise processing
 │    └─ Throttle        stateless-ish throttle math (ramps, limiters, curves)
 ├─ inc_encoder (Encoder)  rotor position/speed/frequency from resolver or
 │                          incremental encoder, feeds VehicleControl and FOC
 ├─ pwmgeneration[-sine|-foc].cpp   PWM/current-loop ISR, torque command intake
 └─ Terminal / TerminalCommands (libopeninv)  serial CLI, see terminal_prj.cpp
```

`libopeninv` (git submodule) supplies the shared infrastructure used across all
project files documented here: the `Param` framework, `Terminal`/`TerminalCommands`,
`CanHardware`/`CanMap`/`CanSdo`, `Stm32Scheduler`, `ErrorMessage`, and math/fixed-point
helpers (`my_fp.h`, `my_math.h`). This file references but does not reimplement
any of that — see the submodule for its internals.

### Boot sequence (`main()`)

1. `clock_setup()`, `rtc_setup()` — PLL/peripheral clocks, 10 ms RTC tick.
2. `hwRev = io_setup()` — hardware-variant autodetect and pin configuration
   (see `doc/hwinit.md`).
3. `tim_setup()`, `nvic_setup()` — PWM/overcurrent timer and interrupt priorities.
4. `parm_load()` — load saved parameters from flash (libopeninv).
5. `pwmio_setup()` — read/configure the 6 PWM output pins, store the observed
   idle pattern in `Param::pwmio`.
6. Construct `Stm32Scheduler`, `Stm32Can`, `CanMap`, `InverterSdo`; wire
   `VehicleControl::SetCan()` and `TerminalCommands::SetCanMap()`.
7. Register `Ms100Task` (100 ms) and `Ms10Task` (10 ms) with the scheduler.
8. Construct the `Terminal` on USART3; disable TX DMA on `HW_REV1`.
9. `UpgradeParameters()` — one-shot parameter migrations (version stamping,
   `snsm` offset bump, `offthrotregen` sign fix, `potmax` clamp, removal of
   CAN mappings for safety-critical parameters).
10. `Param::Change(PARAM_LAST)` — apply all parameters once at boot; then
    `Param::Change(nodeid)` and `Param::Change(outmode)` explicitly (these two
    have side effects — CAN node ID and pin remap — that the generic
    `PARAM_LAST` pass also triggers, called again here for clarity/ordering).
11. `write_bootloader_pininit()` — write the CRC-checked pin-init block the
    bootloader reads on the next reset.
12. Enter the `while(1)` idle loop: `Terminal::Run()` plus the JSON-over-SDO
    print pump (`TerminalCommands::PrintParamsJson`, triggered by a CAN SDO
    read of index `0x5001`, see `doc/invertersdo.md`).

### Scheduler tasks

Both tasks are driven by `Stm32Scheduler::Run()`, called from the timer ISR
(`tim2_isr`/`tim4_isr`) whose source timer is hardware-variant-dependent. The
scheduler class itself (period bookkeeping, overrun handling) lives in
`libopeninv/src/stm32scheduler.cpp` — FORK_NOTES F15/T10 documents an overrun
resync fix there (see `doc/host-tests.md` for its test coverage); this file only
registers the two task functions and their periods.

- `Ms10Task` reads UDC (`VehicleControl::ProcessUdc()`), updates rotor frequency
  (`Encoder::UpdateRotorFrequency(100)`), reads digital inputs and throttle
  (`VehicleControl::ProcessThrottle()`), and either commands torque
  (`PwmGeneration::SetTorquePercent`) or runs the charger ramp (`RunCharger`)
  depending on `opmode`. It owns the `MOD_OFF → MOD_RUN`/charge state
  transition and the `initWait` startup sequencing (current-offset calibration
  at `initWait == 10`, then PWM enable at `initWait == 0`).
- `Ms100Task` is lower-rate telemetry and cruise-control bookkeeping; it does
  not touch the opmode state machine.

### Parameter application (`Param::Change`)

Called by the `libopeninv` parameter framework on every write, from any source
(terminal `set`, CAN SDO, or the boot-time `PARAM_LAST` sweep). Special-cased
parameters (`canspeed`, `outmode`, `manualid`/`manualiq`, `throtmax`/`throtmin`/
`idcmin`/`idcmax`/`offthrotregen`, `nodeid`) get narrow, fast handlers. Everything
else falls into the `default:` branch, which re-derives current limit, pole-pair
ratio, (FOC-only) controller gains/motor parameters/modulation limit, encoder
mode, and the full set of `Throttle::` static fields from their backing
parameters.

## Stability

Compile-checked (`make` and `make CONTROL=FOC`) and covered indirectly by the
host test suite (`Throttle`/`VehicleControl` unit tests). Not hardware-validated
in this fork — see FORK_NOTES.md.
