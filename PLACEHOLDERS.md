# stm32-sine — PLACEHOLDER_HW registry

Hardware-gated values / semantics that ship as **honest guesses**, tagged with a
grep-able `PLACEHOLDER_HW` comment at the site and resolved at bench bring-up.
A placeholder must never be presented as a calibrated/measured value.

Grep the tree for the tag: `grep -rn PLACEHOLDER_HW include src`.

| # | Site (file:line) | What it is | Current placeholder | How the real value is measured at bring-up |
|---|---|---|---|---|
| P1 | `include/param_prj.h` (`iqtimeout` TESTP_ENTRY, id 172) | T21 command-silence timeout: RTC ticks (10 ms each) of manualiq-command silence before firmware zeroes `manualiq`. `0` = disabled. | default `20` (= 200 ms) — a guess: several host control ticks, generous enough to ride out a GC pause yet well under the 500 ms GPIO/canio floor. | On the bench, measure the host `ControlLoop` command cadence (period + worst-case jitter/GC pause) at `dyno/safety/` integration. Set `iqtimeout` to ~3-5x the nominal command period, and strictly below the hardware GPIO dead-man window so this backstop always acts first. Confirm no false trips during a normal run. |
| P2 | `src/vehiclecontrol.cpp` (`VehicleControl::NoteManualCmd`, T21) | CAN-silence *semantics*: T21 assumes the host rewrites `manualiq` every control tick, so **absence of a manualiq/manualid SDO write == host silence**. baseline §8 flags the exact CAN-silence semantics as `[BENCH]`. | Assumption only (no numeric constant): "the command channel is refreshed periodically even in steady-state torque." | Confirm on the bench that the host `ControlLoop` writes `manualiq` every tick even when the commanded value is unchanged (steady torque). If the host instead holds a static `manualiq` between changes, this detection signal is wrong and must move to a dedicated heartbeat SDO write (the rejected-for-now alternative) — see the T21 worklist entry. Verify against the real host before relying on this backstop. |

## Notes
- Both P1 and P2 belong to T21 (`manualiq` auto-zero on command-silence), which
  is **opt-in behind the build flag `MANUALIQ_CMD_TIMEOUT` (`make CMD_TIMEOUT=1`),
  default OFF**. With the flag off these placeholders are compiled out entirely
  (flag-off firmware is byte-identical to `fixes`), so nothing bogus ships in the
  default binary — the placeholders only matter once the feature is enabled for
  bench bring-up.
- T21 is defence-in-depth **behind** the hardware GPIO dead-man heartbeat chain,
  which remains the safety floor regardless of these values.
