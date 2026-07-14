# Getting started

See [`overview.md`](overview.md) for what this firmware does and how the
fork relates to upstream. This doc covers build, flash, and first spin.

## Toolchain

You need the `arm-none-eabi` GCC toolchain
(https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads). On
Ubuntu:

```
sudo apt-get install git gcc-arm-none-eabi
```

## Fetch dependencies

The only external dependency is `libopencm3` (vendored as a fork, pinned as a
submodule). From the repo root:

```
make get-deps
```

This runs `git submodule update --init` and builds `libopencm3`.

## Build

```
make              # SINE build, for induction motors — output stm32_sine.{bin,hex}
CONTROL=FOC make  # FOC build, for synchronous motors — output stm32_foc.{bin,hex}
```

`make` alone defaults to `CONTROL=SINE`. Both builds link against the same
object directory (`obj/`) and share most translation units; only
`pwmgeneration-sine.o` vs. `pwmgeneration-foc.o`/`foc.o` differ (see the
`ifeq ($(CONTROL), ...)` blocks in the `Makefile`).

**Switching `CONTROL=` requires a clean build.** The `Makefile` does not
track the `CONTROL` value as a build input, so switching between
`CONTROL=FOC` and `CONTROL=SINE` (in either direction) without an
intervening `make clean` links stale `.o` files from the other variant and
fails with errors like `undefined reference to
PwmGeneration::SetControllerGains` / `FOC::SetMotorParameters`. This is a
pre-existing upstream issue, not fork-specific. Always run `make clean`
before rebuilding with a different `CONTROL` value.

Run the CONTROL-appropriate `Test` target from the repo root if you want to
also build the host test suite; see
["Running the host test suite"](#running-the-host-test-suite) below.

## Flashing

The README documents three ways to get the built image onto the board:

- A JTAG/SWD adapter. The `Makefile` has a `flash` target that drives
  OpenOCD (`make flash`); it assumes an Olimex STM32-H103-style adapter setup
  (see the `OPENOCD_*` variables in the `Makefile`) — adjust those for your
  adapter/board.
- The `updater.py` script (part of the broader openinverter tooling, not in
  this repo).
- The ESP8266 web interface, if your board carries one.

This repo doesn't document adapter wiring or the web interface's setup —
follow https://openinverter.org/docs for board-specific instructions.

## First boot

### Talking to the board

Two interfaces exist, both built on the same `TermCmds` command table
(`src/terminal_prj.cpp`): a serial terminal, and (on boards with one) the
ESP8266 web interface. Available commands:

| Command | Effect |
|---|---|
| `set` | Set a parameter value |
| `get` | Read a parameter value |
| `flag` | Read/set a parameter's bit flags |
| `stream` | Stream parameter values (text) |
| `binstream` | Stream parameter values (binary) |
| `json` | Print all parameters as JSON |
| `can` | Configure/inspect CAN parameter mapping |
| `save` | Save parameters to flash |
| `load` | Load parameters from flash |
| `reset` | Reset the MCU |
| `defaults` | Reset parameters to their defaults and apply them |
| `stop` | Set `opmode` to `MOD_OFF` — halt the inverter |
| `start <mode>` | SINE build only — set `opmode` directly from the terminal; not implemented in the FOC build (see message in `src/terminal_prj.cpp`) |
| `serial` | Print the MCU's unique ID |
| `errors` | Print the error log |

`save`, `load`, `defaults`, and `reset` are blocked while the motor is
running (`opmode != MOD_OFF`), to prevent a mid-run parameter reload from
desyncing the control loop (see `TerminalCommands::IsSaveEnabled()` and
[`troubleshooting.md`](troubleshooting.md#defaults--load-terminal-behavior)).

### Opmode / start-stop model

`Param::opmode` (`include/param_prj.h`, `enum _modes`) is the state machine
driving everything: `MOD_OFF`, `MOD_RUN`, `MOD_MANUAL`, `MOD_BOOST`,
`MOD_BUCK`, `MOD_SINE`, `MOD_ACHEAT`. `PwmGeneration::SetOpmode()`
(`src/pwmgeneration.cpp`) reconfigures the PWM timer and output enables for
the new mode; `Ms10Task()` in `src/stm32_sine.cpp` is what actually decides
each 10 ms tick whether to move `opmode` out of `MOD_OFF` into `MOD_RUN` (or
into `MOD_BOOST`/`MOD_BUCK` for charging).

The gating conditions checked before the DC contactor closes and the motor
is allowed to run (see `Ms10Task()` and `STAT_*` flags) are, all must hold:
- emcystop input inactive (not tripped)
- motor-protection input inactive
- throttle not pressed (`STAT_POTPRESSED` clear)
- DC bus voltage ≥ `udcsw`
- DC bus voltage < `udclim`
- a brake pedal has been seen at least once since boot (`seenBrakePedal`)

With those satisfied, `opmode` moves to `MOD_RUN` when `din_start` (or
`manualstart`, or a `tripmode == TRIP_AUTORESUME` auto-restart after a trip)
is asserted. See [`troubleshooting.md`](troubleshooting.md#motor-wont-start)
for what to check if it won't.

### Minimum parameters before first spin

Exact parameter names/ranges are in `include/param_prj.h`; the full list is
in [`parameters.md`](parameters.md). At minimum, before commanding a spin:

- `polepairs` — motor pole pairs (both builds).
- Position sensing, one of:
  - `encmode` + `numimp` (ppr) — incremental encoder.
  - `respolepairs` — resolver pole pairs (only meaningful if `encmode`
    selects a resolver). Don't leave it at its default of 1 unchecked: a
    wrong value scales the electrical angle and the motor won't turn
    correctly even though everything else is right.
- `il1gain` / `il2gain` — current sensor gain, both builds; wrong gain skews
  the overcurrent/current-limit thresholds derived from `ocurlim`.
- FOC build only: `iqkp`, `idkp`, `curki` — current-loop PI gains. Left at
  defaults these are a reasonable starting point on some motors but not all;
  tune conservatively.
- `udcsw` — DC bus voltage above which the contactor is allowed to close.
- `udclim` — DC bus voltage above which the inverter shuts down
  (overvoltage).

Verify hardware-specific pinswap/sense settings (`pinswap`, `snshs`, `snsm`)
against your board revision before the first attempt.

## Safety notes (this firmware, not generic)

- **Precharge.** The DC contactor (`dcsw_out`) doesn't close until bus
  voltage crosses `udcsw`. Precharge is expected to bring the bus up to
  `udcsw / 2` within 5 s (`PRECHARGE_TIMEOUT`, `src/vehiclecontrol.cpp`); if
  it hasn't, `ERR_PRECHARGE` is posted and the precharge output is dropped.
- **Overcurrent (hardware trip).** `ocurlim` (A) sets the hardware
  comparator threshold that trips `tim1_brk_isr`
  (`src/pwmgeneration.cpp`) and immediately forces `opmode = MOD_OFF`. A
  negative `ocurlim` used to invert the trip thresholds (fixed in this
  fork — F24); the code now takes `ABS(ocurlim)`. On `HW_PRIUS` hardware,
  `ocurlim == -1` is a sentinel that disables the break input entirely —
  don't set it to `-1` on other hardware.
- **Current limit (soft, SINE build).** `iacmax` caps AC current by pulling
  back the amplitude/slip setpoint before the hardware trip is reached
  (`PwmGeneration::GetIlMax` derivative in `src/pwmgeneration-sine.cpp`);
  `iacmax == 0` disables it.
- **Desat / gate-driver faults.** `tim1_brk_isr` distinguishes overcurrent
  from a gate-driver desaturation fault, motor-protection trip, and
  emergency-stop input by reading `desat_in`/`mprot_in`/`emcystop_in` at
  trip time (hardware-revision-dependent; some revisions don't wire all
  three). Any of these forces `opmode = MOD_OFF` and sets the fault output.
- **Overvoltage.** If filtered bus voltage exceeds `udclim`, the inverter is
  forced off (`ERR_OVERVOLTAGE`); if the motor is stationary at the time,
  the DC switch and precharge output are also opened, since a stationary
  overvoltage is assumed to come from outside the inverter (e.g. charger).

See [`troubleshooting.md`](troubleshooting.md) for the full fault code table.

## Running the host test suite

Fork-added; not part of upstream. Requires only a host `gcc`/`g++`, no ARM
toolchain:

```
cd test
make
./test_sine
```

This exercises the control classes with no hardware: PI controller, FOC
math, CAN filter packing, scheduler overrun logic, throttle, anti-cogging,
q-axis clamp, regen taper hold, and more (see `test/test_*.cpp`). CI runs
this alongside both firmware builds — see
[`../FORK_NOTES.md`](../FORK_NOTES.md) for the fork's verification
convention.
