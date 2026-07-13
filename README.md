## About this fork

This is the moltenmosfet dyno-absorber fork of jsphuebner/stm32-sine (Open
Inverter). It drives a Nissan Leaf EM57 motor as the absorber on a chassis
dynamometer.

`dyno-main` is the integration branch. It was created off the pinned upstream
baseline commit `1dfab85`. Fix branches (`fix/T<nn>-…`) merge into `dyno-main`
after review.

The firmware is split across two repos. `libopeninv` is vendored as a git
submodule and is forked in the same way, at
[moltenmosfet/libopeninv](https://github.com/moltenmosfet/libopeninv); its
`dyno-main` was created off pin `78e3f72`. The `libopencm3` submodule stays at
upstream's pin and is not forked.

Most changes here are tagged as upstream candidates and will be offered to
jsphuebner as a PR series. A smaller set of changes are fork-only policy
decisions specific to running this firmware on a dyno absorber — the
low-speed torque-limit behavior and a deadtime compensation experiment among
them — plus the documentation itself. Per-change notes and rationale are in
[FORK_NOTES.md](FORK_NOTES.md).

Nothing in this fork is bench-validated on hardware yet (the dyno hardware is
still being built). Changes are host-tested and compile-checked only.

## Upgrades in this fork:

- **Low-speed / low-torque control quality** — the PI controller's integral
  term no longer moves in coarse steps (64-bit fix), optional dead-time
  compensation, the low-speed q-axis clamp is now a configurable policy with
  hysteresis and slew instead of a hard polarity switch, and regen torque
  tapers smoothly through the standstill deadband instead of cutting on/off.
- **Encoder and angle handling** — fixed a read-modify-write race between the
  PWM and frequency-update interrupts, clamped single-channel pulse
  interpolation so the synthesized angle can't overshoot, and fixed a 16-bit
  wraparound in the anti-cogging feedforward.
- **CAN / SDO robustness** — three receive-filter setup bugs fixed (dropped
  filters, wasted mask banks, accidental acceptance of CAN ID 0), incoming SDO
  frames length-checked, and parameter reload / defaults / reset commands are
  blocked while the motor is running.
- **Scheduler reliability** — a task that overruns its period now resyncs
  immediately instead of silently stalling for up to ~650 ms.
- **Overcurrent and numeric safety** — the current-magnitude overflow fix from
  upstream issue #29 is actually wired in, the modulation-limit calculation
  can no longer underflow when `modmax` is lowered at runtime, and a negative
  `ocurlim` can no longer invert the trip thresholds.
- **Terminal and SINE-build fixes** — `defaults` now applies the values it
  loads, `start` works again in the SINE build, and an RMS calculation no
  longer divides by zero below 1 Hz.
- **Host-side test harness** — the control classes (PI controller, FOC math,
  CAN filter packing, scheduler overrun logic, and more) now run under a
  native host test suite; every change is host-tested and both `CONTROL=FOC`
  and `CONTROL=SINE` firmware builds are compile-checked.

---

[![Build status](../../actions/workflows/CI-build.yml/badge.svg)](../../actions/workflows/CI-build.yml)

# stm32-sine
Main firmware of the Huebner inverter project
This firmware runs on any revision of the "Huebner" hardware https://github.com/jsphuebner/inverter-hardware as well as any derivatives as the Open Source Tesla controller https://github.com/damienmaguire

# Goals
The main goal of this firmware is well-drivable control of electric 3-phase motors with as little software complexity as possible. We do not rely on virtual control methods such as FOC (field oriented control) or DTC (direct torque control). This makes tuning more intuitive, as only real physical quantities are parametrized.
The same principle is applied to the hardware design, keeping component count low and therefor minimize cost and failure modes.
To fine tune the driving experience and adapt to different flavours of power stages, over 60 parameters can be customized.

# Motor Control Concept
The idea is that the dynamics of any 3-phase asynchronous motor are controlled by the amplitude of the sythesized sine wave and its frequency offset to the rotor speed (slip). 
For 3-phase synchronous motors a similar control method did not prove practical. Therefor a FOC version of the software has been created. It shares 95% of the code.

# Inverter charging
A unique feature of this software is to re-purpose the drivetrain hardware as a programmable battery charger. One of the motor phase windings is being used as a high current capable inductor and one of the phase switches as a buck or boost converter. This has practically proven to replace a separate charging unit and further reduce complexity of electric vehicles.

# Further reading
A comprehensive guide to the Huebner inverter system can be found here: https://openinverter.org/docs

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
On Ubuntu type

`sudo apt-get install git gcc-arm-none-eabi`

The only external depedency is libopencm3 which I forked. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-sine by typing

`make`

or

`CONTROL=FOC make`

to build the FOC version for synchronous motors.

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface
