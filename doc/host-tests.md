# Host test harness (`test/`)

## Description

`test/` is a native (x86, no ARM toolchain needed) unit test build of selected
firmware source files, linked against a small set of hand-written libopencm3
and `CanHardware` stubs instead of real hardware. Build/run:

```
cd test && make && ./test_sine
```

`make` alone builds `test_sine`; there's no separate `test` target — running
the binary is a manual second step. `test/Makefile` always compiles with
`-DCTRL_FOC=1 -DCTRL_SINE=0 -DCONTROL=CTRL_FOC` — the host suite only ever
builds the FOC configuration; there is no host build of the SINE-only code
paths (e.g. `stm32_sine.cpp`'s SINE-only `uac`/field-weakening blocks aren't
compiled here, and `stm32_sine.cpp`/`vehiclecontrol.cpp`'s ISR/`main()` code
isn't linked into the test binary at all — only `vehiclecontrol.cpp`,
`throttle.cpp`, and the `libopeninv` sources listed in `OBJS` are).

Object list (`test/Makefile` `OBJS`, via `VPATH = ../src ../libopeninv/src`):
`vehiclecontrol.cpp`, `throttle.cpp`, `sine_core.cpp`, `temp_meas.cpp`,
`fu.cpp`, `my_fp.c`, `my_string.c`, `params.cpp`, `picontroller.cpp`, `foc.cpp`,
`canfilterpack.cpp`, `stm32scheduler.cpp` — real production code, compiled
unmodified — plus the two stub translation units
(`stub_canhardware.cpp`, `stub_libopencm3.c`) and one test file per area.

This harness itself predates the fork in skeleton form (`test_main.cpp`,
`test.h`, `test_fp.cpp`, `test_fu.cpp`, `stub_canhardware.cpp`/`.h`, and the
basic `Makefile` shape were already present upstream) but this fork added
most of its current coverage: `stub_libopencm3.c`/`.h` (new stub surface for
CAN filter banks and timer registers), and the test files
`test_picontroller.cpp`, `test_foc.cpp`, `test_getilmax.cpp`, `test_qclamp.cpp`,
`test_regentaperhold.cpp`, `test_anticog.cpp`, `test_canfilterpack.cpp`,
`test_stm32scheduler.cpp`, plus substantial expansion of the pre-existing
`test_throttle.cpp` and `test_vcu.cpp`. See FORK_NOTES.md's task table (T1–T20)
for which finding (F1–F24) each test file backs.

## Why?

Most of this firmware's actual defects (per FORK_NOTES.md's F1–F24 survey)
live in plain arithmetic and state machines — PI controller gain math, angle
wraparound, CAN filter-bank packing, scheduler overrun handling, throttle
limiter chains — that don't need real STM32 peripherals to exercise. Splitting
that logic so it's reachable from a native x86 build lets it be tested on
every commit (`test_sine` runs in CI, see `GITHUB_RUN_NUMBER` handling in the
Makefile) without hardware in the loop, and turns silent regressions in
low-level math (the exact class of bug F2, F6, F7, F17, F19–F23 are) into a
build failure instead of a field report.

## Drawbacks

- FOC-only build config means SINE-specific code is never compiled or run
  by this harness — a regression confined to the SINE build path would not be
  caught here.
- `main()`/ISR entry points, `hwinit.cpp`, and anything that touches real
  GPIO/ADC/timer registers beyond what's stubbed is untestable in this
  harness by construction — see `doc/hwinit.md`. The stub timer functions
  are explicit no-ops or fixed values (e.g. `crc_calculate()` always returns
  `0xaa55`, `desig_get_flash_size()` always returns `8`); tests that would
  depend on real CRC or flash-size values can't be written against this stub
  as-is.
- Some pure-logic code was refactored specifically to become host-testable
  (e.g. `canfilterpack.cpp` split out of the CAN filter setup in
  `libopeninv/src/stm32_can.cpp` for T9, `Stm32Scheduler::CheckOverrun()`
  split out as a pure function for T10) — a deliberate, documented trade-off
  (FORK_NOTES T9 row: "PR 5 may inline the split back if upstream prefers"),
  not free.
- No coverage tooling is wired up; "covered" here means "a test file exists
  and asserts on it," not a measured line/branch coverage number.

## What it stubs

- **`stub_canhardware.cpp`/`.h`** (`libopeninv/test/`, pre-fork): a minimal
  `CanHardware` subclass (`CanStub`) plus free functions backing
  `CanHardware`'s pure-virtual-adjacent surface. `AddCallback()` captures the
  registered callback in a global `vcuCan`; `RegisterUserMessage()` records
  the registered ID in `vcuCanId`; `HandleRx()` just forwards to `vcuCan`.
  Lets tests inject CAN frames directly and inspect what a real send would
  have contained (`CanStub::m_data`/`m_canId`/`m_len`).
- **`stub_libopencm3.c`/`.h`** (fork-added): flash (`flash_unlock/lock/
  set_ws/program_word/erase_page` — all no-ops), `desig_get_flash_size`
  (fixed `8`), `crc_calculate` (fixed `0xaa55`), a recording stub for the
  three `can_filter_id_*_init` functions (captures bank number, FIFO, enable
  flag, and raw id/mask arguments into `stub_filter_calls[]` for T9's tests
  to inspect what got programmed), and a settable/recording timer stub
  (`stub_timer_counter`, `stub_timer_oc_value[]`, plus no-op stand-ins for
  `timer_enable_preload`/`timer_direction_up`/`timer_set_prescaler`/etc. that
  exist only so `stm32scheduler.cpp` links) for T10's overrun tests. Comments
  in the file are explicit about which stubs are load-bearing for assertions
  versus which are link-only no-ops.

## What it covers

- `test_fp.cpp`, `test_fu.cpp`: fixed-point math (`atan2`) and the field-
  weakening/boost lookup (`fu.cpp`), pre-fork.
- `test_throttle.cpp`: throttle curve linearity, brake-pedal override, dual-
  throttle range fallback, and (fork addition) that
  `FrequencyLimitCommand`/`FrequencyLimitCommandFw` no longer share IIR
  filter state — see `doc/throttle.md`.
- `test_vcu.cpp`: the CAN control-frame path in `VehicleControl` — CRC/
  sequence-counter fields present, and (fork additions) sequence-error
  detection/recovery and brake-light hysteresis behavior — see
  `doc/vehiclecontrol.md`.
- `test_picontroller.cpp`, `test_foc.cpp`, `test_getilmax.cpp`: PI controller
  proportional/clamp/windup/integral-granularity behavior (F2, F7), FOC
  Park/Clarke invariants, MTPA symmetry, deadtime compensation sign mapping
  and deadband (F10), and current-magnitude overflow handling (F17) — these
  exercise `libopeninv/src/picontroller.cpp` and `libopeninv/src/foc.cpp`,
  not files documented individually elsewhere in `doc/`.
- `test_qclamp.cpp`: the low-speed q-axis voltage clamp policy (F1) in
  `include/qclamp.h`.
- `test_regentaperhold.cpp`: `RegenTaperHold` (F4) — see `doc/vehiclecontrol.md`.
- `test_anticog.cpp`: anti-cogging angle-wraparound fix (F5).
- `test_canfilterpack.cpp`: CAN receive filter-bank packing across the
  16/32-bit, list/mask, standard/extended combinations (F12).
- `test_stm32scheduler.cpp`: scheduler deadline-overrun resync behavior
  across counter wraparound (F15).

None of these tests exercise `src/stm32_sine.cpp`, `src/invertersdo.cpp`,
`src/terminal_prj.cpp`, `src/temp_meas.cpp`, or `src/hwinit.cpp` directly — see
each of those files' own doc for what validates them (compile-check and/or
code review only, in most cases).

## Stability

`test_sine` builds and passes (`All tests passed`, 0 failed assertions; some
tests assert in loops so total assertion count runs into the tens of
thousands) across the 13 `test_*.cpp` files in `test/`, as of this writing,
compiled with the toolchain available in this checkout. This
is a host-only, simulated-peripheral test — it validates arithmetic and state
machines, not real hardware timing or electrical behavior. No part of this
fork is hardware-validated; see FORK_NOTES.md.
