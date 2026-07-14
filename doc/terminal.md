# Terminal commands

## Description

`src/terminal_prj.cpp` defines the `TermCmds[]` table passed to `libopeninv`'s
`Terminal` class at construction (`Terminal t(USART3, TermCmds);` in
`stm32_sine.cpp`) and the small number of command handlers that are
project-specific rather than generic library behavior. Built into both SINE
and FOC variants; `StartInverter` is the only handler that's `#if CONTROL`-gated
internally.

Command table:

| command | handler | source |
|---|---|---|
| `set` | `TerminalCommands::ParamSet` | libopeninv |
| `get` | `TerminalCommands::ParamGet` | libopeninv |
| `flag` | `TerminalCommands::ParamFlag` | libopeninv |
| `stream` | `TerminalCommands::ParamStream` | libopeninv |
| `binstream` | `TerminalCommands::ParamStreamBinary` | libopeninv |
| `json` | `TerminalCommands::PrintParamsJson` | libopeninv |
| `can` | `TerminalCommands::MapCan` | libopeninv |
| `save` | `TerminalCommands::SaveParameters` | libopeninv |
| `load` | `TerminalCommands::LoadParameters` | libopeninv |
| `reset` | `TerminalCommands::Reset` | libopeninv |
| `defaults` | `LoadDefaults` | this file |
| `stop` | `StopInverter` | this file |
| `start` | `StartInverter` | this file |
| `serial` | `PrintSerial` | this file |
| `errors` | `PrintErrors` | this file |

Only `defaults`, `stop`, `start`, `serial`, `errors` are defined in this file;
the rest are generic parameter/CAN-map/persistence commands implemented in
`libopeninv/src/terminalcommands.cpp` — see that submodule for their behavior
(not documented here). Parameters touched directly here: `opmode` (via
`Param::SetInt`, `stop`/`start`), and implicitly every parameter via
`Param::LoadDefaults()` + `Param::Change(PARAM_LAST)` (`defaults`).

## Why?

`libopeninv`'s `Terminal`/`TerminalCommands` supply the generic CLI plumbing
(line parsing, parameter get/set/stream, CAN mapping, save/load/reset) shared
across `libopeninv`-based firmwares. This file only adds the handful of
commands that need to reach into this firmware's own state (`opmode`,
`PwmGeneration`) or hardware (`DESIG_UNIQUE_ID*`), the same split as
`invertersdo.cpp` for the CAN SDO side (see `doc/invertersdo.md`).

## Drawbacks

- `StartInverter` is only implemented for the SINE build; the FOC build's
  handler just prints a message pointing at an openinverter.org forum thread
  instead of doing anything — starting the FOC firmware from the terminal is
  not supported.
- `LoadDefaults`/`StopInverter`/`StartInverter` all mutate `Param::opmode` or
  full parameter state directly from a serial command with no
  authentication — anyone with UART access has full control. This mirrors the
  existing CAN SDO trust model (see `doc/invertersdo.md`) rather than being a
  new gap.
- No host test exists for this file — it's not part of `test/`'s object list;
  the running-state gate on `defaults` is exercised only by code review and
  the parallel `libopeninv` `load` gate's design symmetry, not by an automated
  test.

## Architecture

`Terminal::Run()` (libopeninv, called from the idle loop in `main()`) parses a
line, matches the first token against `TermCmds[]`, and calls the matching
handler with the remainder of the line as `arg`.

- `PrintSerial` reads the three `DESIG_UNIQUE_ID*` registers (STM32 factory
  unique ID) and prints them colon-separated.
- `PrintErrors` calls `ErrorMessage::PrintAllErrors()` (libopeninv).
- `StopInverter` unconditionally sets `Param::opmode = 0`.
- `StartInverter` (SINE only): parses `arg` as an integer, and if it's a valid
  mode (`< MOD_LAST`), sets `Param::opmode` and calls
  `PwmGeneration::SetOpmode()` directly — bypassing the same `Ms10Task`
  interlocks that `invertersdo.cpp`'s remote START bypasses (see
  `doc/invertersdo.md` and `doc/stm32_sine.md`).
- `LoadDefaults` gates on `TerminalCommands::IsSaveEnabled()` before calling
  `Param::LoadDefaults()`; see Fork additions below.

### Fork additions

- **`defaults` no-op fix (F18/T16).** Originally `LoadDefaults()` called
  `Param::LoadDefaults()` and printed "Defaults loaded" without ever calling
  `Param::Change(PARAM_LAST)` — the in-memory parameter values were reset, but
  nothing recalculated the runtime state derived from them (`Throttle::`
  fields, encoder mode, FOC gains, etc.), so the reset had no observable
  effect until an unrelated parameter write happened to trigger the
  recalculation. Now it calls `Param::Change(Param::PARAM_LAST)` immediately
  after `LoadDefaults()`, mirroring the CAN SDO `DEFAULTS` command path
  (`libopeninv/src/sdocommands.cpp`), and is gated on
  `TerminalCommands::IsSaveEnabled()` — same "not in RUN" lockout as `save`
  and (now) `load`/the CAN SDO LOAD/DEFAULTS commands (see
  `doc/invertersdo.md`) — printing "Will not load defaults in run modes,
  please stop before loading!" if blocked.
- **`start` fixed-point bug (F19/T17).** `StartInverter` used to call
  `PwmGeneration::SetOpmode(FP_TOINT(val))` where `val` is already a plain
  `int` from `my_atoi()` — `FP_TOINT` treats its argument as a Q-format
  fixed-point value and right-shifts it, so for the small integer mode values
  in use (`MOD_OFF`=0 through low single digits) this silently produced
  `MOD_OFF` (0) for most nonzero inputs. Fixed by removing the erroneous
  `FP_TOINT()` call; `SetOpmode(val)` now receives the plain integer mode.

## Stability

Not covered by the host test suite; compile-checked in both variants. Not
hardware-validated in this fork.
