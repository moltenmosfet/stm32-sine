# Overview

## Aim

This firmware's goal is well-drivable control of electric 3-phase motors with
as little software complexity as possible. It deliberately avoids virtual
control methods like FOC or DTC for induction motors, controlling the
synthesized sine wave's amplitude and its frequency offset to rotor speed
(slip) instead — real physical quantities that are easier to tune. Over 60
parameters are exposed to adapt the firmware to different motors and power
stages.

## Two build variants

The codebase produces two firmware images from the same tree, selected at
build time:

- **`make`** — the SINE build, for 3-phase asynchronous (induction) motors.
  Open-loop amplitude + slip-frequency control via `SineCore`.
- **`CONTROL=FOC make`** — the FOC build, for 3-phase synchronous motors, for
  which the slip-based approach doesn't work. Closed-loop field-oriented
  control via `FOC` and two `PiController` instances.

The two variants share about 95% of the code. Shared infrastructure —
parameter handling, the opmode state machine, CAN/SDO, the scheduler, encoder
handling, throttle processing — lives in common source files; only the
control-law-specific pieces (`src/pwmgeneration-sine.cpp` vs.
`src/pwmgeneration-foc.cpp` / `src/foc.cpp`) differ. See
[`doc/pwmgeneration.md`](pwmgeneration.md) for the split in detail.

## Inverter charging

A unique feature of this software: it repurposes the drivetrain hardware as a
programmable battery charger. One motor phase winding is used as a high-current
inductor, and one phase switch as a buck or boost converter (`MOD_BOOST` /
`MOD_BUCK` in the opmode state machine). This has replaced a separate charging
unit in practice and reduces overall vehicle complexity.

## Supported hardware

The firmware runs on any revision of the Huebner inverter board
(https://github.com/jsphuebner/inverter-hardware) and derivatives, such as the
open-source Tesla controller (https://github.com/damienmaguire). Hardware
variants are distinguished at runtime by `hwRev` (`HWREV`), which gates
board-specific I/O behavior (e.g. desat/emcystop/mprot pin wiring, ADC
channels) throughout `src/vehiclecontrol.cpp` and `src/pwmgeneration.cpp`.

For hardware wiring and board documentation, see
https://openinverter.org/docs.

## This fork

`stm32-sine` here is a bug-fix fork of upstream jsphuebner/stm32-sine. A
source-level review of the FOC control path, the CAN/SDO stack, the
scheduler, and the SINE build produced 24 findings; all 24 have been fixed on
the `fixes` integration branch. See [`../README.md`](https://github.com/moltenmosfet/stm32-sine/blob/fixes/README.md) for the
fork summary and [`../FORK_NOTES.md`](https://github.com/moltenmosfet/stm32-sine/blob/fixes/FORK_NOTES.md) for the full findings
glossary and per-task rationale.

The fork adds no new control behavior of its own. The only dyno-specific
policy sits behind two optional parameters, both defaulting to stock
behavior:

- `qlimfrq` (Hz, default 30) — configures the low-speed q-axis voltage clamp
  policy; `0` relaxes the clamp for dyno-style low-speed operation.
- `dtcomp` (dig, default 0) — dead-time compensation; `0` is off.

Nothing in this fork has been validated on real hardware yet — changes are
host-tested (see [`getting-started.md`](getting-started.md#running-the-host-test-suite))
and compile-checked only, pending the dyno hardware build.

## Further reading

- [`getting-started.md`](getting-started.md) — build, flash, first spin.
- [`troubleshooting.md`](troubleshooting.md) — fault codes and bring-up
  failures.
- https://openinverter.org/docs — hardware and wiring guide.
- https://openinverter.org/forum — community support.
