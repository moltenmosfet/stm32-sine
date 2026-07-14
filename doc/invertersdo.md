# InverterSdo (CAN SDO interface)

## Description

`src/invertersdo.cpp` / `include/invertersdo.h`. `InverterSdo` is a thin,
project-specific subclass of `libopeninv`'s `CanSdo` (`libopeninv/src/cansdo.cpp`,
`libopeninv/include/cansdo.h`). It overrides one virtual —
`ProcessUserSpaceSdo(SdoFrame*)` — to add two inverter-specific remote commands
on top of `CanSdo`'s generic CANopen-like SDO protocol (parameter
read/write-by-ID, CAN-map add/remove, save/load/reset/defaults, serial number,
JSON parameter dump). Built into both SINE and FOC variants unconditionally.

`InverterSdo` itself handles exactly two writes to SDO index `SDO_INDEX_COMMANDS`
(`0x5002`, defined in `libopeninv/include/sdocommands.h`):
- subindex 4 (`START_COMMAND_SUBINDEX`): if `data < MOD_LAST`, sets
  `Param::opmode` directly to the requested mode; otherwise replies
  `SDO_ABORT` / `SDO_ERR_RANGE`.
- subindex 5 (`STOP_COMMAND_SUBINDEX`): sets `Param::opmode = 0`.

Everything else at index `0x5002` (save/load/reset/defaults/clear-CAN, subindices
0–3 and 6) falls through to `CanSdo::ProcessSDO()` and from there to
`SdoCommands::ProcessStandardCommands()` (`libopeninv/src/sdocommands.cpp`).
All other SDO indices (`0x2000` parameter-by-index, `0x21xx` parameter-by-UID,
`0x2200` parameter flags, `0x3000`/`0x3001`/`0x31xx` CAN mapping, `0x5000`
serial, `0x5001` JSON string print, `0x5003`/`0x5004` error log) are handled
entirely inside `libopeninv` and are not documented here — see that submodule.

Related artifact: `misc/oi-inverter.dbc` describes the *vehicle control* CAN
frame (pot, pot2, cruise/start/brake/forward/reverse/bms bits, cruisespeed,
regenpreset) — this is the raw CAN message `VehicleControl::CanReceive()`
parses (see `doc/vehiclecontrol.md`), not the SDO protocol. It's the only CAN
DBC artifact in this repo and is grouped here because it's the closest thing to
protocol documentation for this project's CAN surface.

## Why?

`CanSdo` is generic, shared infrastructure — reusable across any `libopeninv`-based
firmware. Remote start/stop is inverter-specific behavior (it directly drives
`opmode`, which only makes sense in this firmware's state machine), so it's
kept out of the shared library via the `ProcessUserSpaceSdo` virtual hook,
which `CanSdo::HandleRx()` calls before falling back to its own generic
`ProcessSDO()`.

## Drawbacks

- Remote START bypasses the `Ms10Task` start interlocks entirely. Setting
  `Param::opmode` directly here skips the `STAT_*` gate in `Ms10Task`
  (emergency stop, motor-protection switch, UDC window, brake-pedal-seen
  latch, throttle-not-pressed) that a normal `din_start`/`manualstart` trigger
  goes through, and does not close the DC contactor (`DigIo::dcsw_out`) —
  that only happens inside `Ms10Task` when it observes `newMode != MOD_OFF`.
  `libopeninv/src/sdocommands.cpp` documents this explicitly in a comment:
  sequencing of a remote start (i.e. actually closing the contactor
  first) is the caller's responsibility. This is documented behavior
  (FORK_NOTES F11's second half), not something this fork changed in
  `invertersdo.cpp`.
- No length/range validation beyond the `MOD_LAST` bound on START; a malformed
  frame that reaches `ProcessUserSpaceSdo` with the right index/subindex will
  still change `opmode`.
- No host test exists for `InverterSdo` itself (it isn't part of the
  `test/` object list) — only `libopeninv`'s own `test_cansdo.cpp` covers the
  base class.

## Architecture

`CanSdo::HandleRx(canId, data, dlc)` is registered as a `CanCallback` against
CAN IDs `0x600+nodeId` (requests) and, when acting as an SDO client,
`0x580+remoteNodeId` (replies). Fork addition (F16/T12, in
`libopeninv/src/cansdo.cpp`): frames with `dlc < 8` are dropped immediately —
`data[]` is a fixed 2-word buffer that would otherwise contain stale bytes from
a previous, longer CAN mailbox use if a short frame arrived. On a valid-length
SDO request frame, `HandleRx` calls the virtual `ProcessUserSpaceSdo(sdo)`
first; `InverterSdo`'s override handles index `0x5002` subindex 4/5 and returns
`true` (frame handled, reply already built), otherwise returns `false` and
`HandleRx` falls through to `CanSdo::ProcessSDO()`.

`SdoCommands::ProcessStandardCommands()` (`libopeninv/src/sdocommands.cpp`)
handles the rest of index `0x5002`: SAVE, LOAD, RESET, DEFAULTS, CLEAR_CAN.
Fork addition (F11/T8): SAVE was already gated on
`SdoCommands::saveEnabled`; this fork extended the same gate to LOAD and
DEFAULTS, because both end in `Param::Change(PARAM_LAST)`, which re-derives
encoder mode, pole-pair ratio, and other run-time state that must not change
out from under a running control loop. Both now reply `SDO_ABORT` /
`SDO_ERR_GENERAL` if `saveEnabled` is false. RESET is deliberately left
unconditional — it lands in the bootloader's pin-init and is considered
survivable while running. `saveEnabled` itself is toggled by
`SdoCommands::EnableSaving()`/`DisableSaving()`, called from `Ms10Task` in
`stm32_sine.cpp` on the `MOD_OFF` / just-started transitions (see
`doc/stm32_sine.md`); the same accessor (`TerminalCommands::IsSaveEnabled()`)
gates the terminal `load`/`defaults` commands (see `doc/terminal.md`) with
identical wording, so CAN and serial present the same running-state lockout.

## Stability

Compile-checked in both variants. The running-state gate (LOAD/DEFAULTS) was
review-verified rather than covered by an automated test — FORK_NOTES notes an
automated harness for this path wasn't judged cost-effective. Not
hardware-validated in this fork.
