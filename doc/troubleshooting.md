# Troubleshooting

Grounded in this firmware's code. Where the code doesn't answer the
question, this doc links to https://openinverter.org/docs or
https://openinverter.org/forum instead of guessing. See
[`getting-started.md`](getting-started.md) for build/flash/first-boot and
[`overview.md`](overview.md) for what the firmware does.

## Fault / error codes

Codes are defined in `include/errormessage_prj.h` and read with the
`errors` terminal command (`ErrorMessage::PrintAllErrors`,
`src/terminal_prj.cpp`). Each has a class (`ERROR_STOP` / `ERROR_DERATE` /
`ERROR_DISPLAY`) shown as `STOP`/`DERATE`/`WARN` when printed. **Note:**
`ErrorMessage::Post()` (`libopeninv/src/errormessage.cpp`) only logs and
prints — the class label doesn't itself stop or derate anything. Every
call site that needs to stop the inverter or cut torque does that
explicitly alongside the `Post()` call; see the "Triggered by" column.

| Code | Class | Triggered by | What to check |
|---|---|---|---|
| `OVERCURRENT` | STOP | `tim1_brk_isr` (`src/pwmgeneration.cpp`) — hardware break-input trip, once desat/emcystop/mprot have been ruled out (or on `HW_TESLA`, `ocur_in` set). Forces `opmode = MOD_OFF`. | `ocurlim` value and sign, current sensor wiring/gain (`il1gain`/`il2gain`), an actual overcurrent event. |
| `THROTTLE1` | DISPLAY | `VehicleControl::GetUserThrottleCommand` (`src/vehiclecontrol.cpp`) — throttle channel 1 reading is outside `potmin`/`potmax` (± slack) per `Throttle::CheckAndLimitRange`. Only posted while `opmode == MOD_RUN` (`PostErrorIfRunning`). | `potmin`/`potmax` calibration, throttle wiring, pot failure. |
| `THROTTLE2` | DISPLAY | Same function — dual-channel potmode (`POTMODE_DUALCHANNEL`) and channel 2 is out of range, or (in bidir/no-dual mode paths) both channels bad. | `pot2min`/`pot2max`, second throttle channel wiring, `potmode` setting. |
| `CANTIMEOUT` | STOP | Two sites: `VehicleControl::GetDigInputs` when digital-input CAN data hasn't arrived within `CAN_TIMEOUT` and `canIoActive` was set; `GetUserThrottleCommand` when throttle is sourced over CAN (`potmode & POTMODE_CAN`) and no update arrived within `CAN_TIMEOUT` (500 ms). | CAN bus wiring/termination, sender node alive, `canio`/`potmode` CAN-source configuration, bus load. |
| `EMCYSTOP` | STOP | `tim1_brk_isr` — `emcystop_in` inactive (checked on hardware revisions other than `HW_REV3`). | Emergency-stop loop wiring, `hwRev` correctness. |
| `MPROT` | STOP | `tim1_brk_isr` — `mprot_in` inactive (not `HW_BLUEPILL`). | Motor-protection switch wiring/state. |
| `DESAT` | STOP | `tim1_brk_isr` — `desat_in` inactive (not `HW_REV1`/`HW_BLUEPILL`), or (on `HW_TESLA`) `ocur_in` set once emcystop/mprot are ruled out. | Gate driver desaturation fault — check gate drive, short on a phase leg, `hwRev` setting. |
| `OVERVOLTAGE` | STOP | `VehicleControl::ProcessUdc` — filtered DC bus voltage `udcfp` exceeds `udclim`. Forces `opmode = MOD_OFF`; if the motor is stationary, also opens the DC switch and precharge output. | `udclim` setting, actual bus voltage, `udcgain`/`udcofs` calibration. |
| `ENCODER` | DISPLAY | `Encoder::GetPulseTimeFiltered` (`src/inc_encoder.cpp`) — adjacent pulse-time spike ratio > 8x, treated as interference. | Encoder signal integrity/noise, `numimp` (ppr) correctness, encoder wiring/shielding. |
| `PRECHARGE` | STOP | `VehicleControl::ProcessUdc` — bus voltage hasn't reached `udcsw / 2` within `PRECHARGE_TIMEOUT` (5 s) while the precharge output is still active. | Precharge resistor/relay, `udcsw` value, battery contactor sequence. |
| `TMPHSMAX` | DERATE | `VehicleControl::ProcessThrottle` — heatsink temperature `tmphs` exceeds `tmphsmax` (`Throttle::TemperatureDerate` cuts the throttle command). | Cooling, `tmphsmax` setting, heatsink temp sensor (`snshs`) calibration. |
| `CURRENTLIMIT` | DERATE | `PwmGeneration::GetIlMax`-derived soft current limiter (`src/pwmgeneration-sine.cpp`, SINE build) — the amplitude/slip setpoint has been pulled back below the commanded value because AC current is approaching `iacmax`. | `iacmax` setting relative to actual load; expected during hard acceleration, a bug if it's constant at light load. |
| `PWMSTUCK` | DISPLAY | Defined in `errormessage_prj.h` but not posted anywhere in this firmware's current code paths (`ErrorMessage::Post(ERR_PWMSTUCK)` doesn't appear in `src/`). Reserved. | If you see this, it's from a code path this doc doesn't cover — search the binary's actual source for where it's wired. |
| `HICUROFS1` / `HICUROFS2` | DISPLAY | `PwmGeneration::SetCurrentOffset` (`src/pwmgeneration.cpp`) — the measured current-sensor zero-offset for channel 1/2 is outside the expected bipolar range (`CHK_BIPOLAR_OFS`). | Current sensor zero calibration, sensor wiring, `il1gain`/`il2gain`. |
| `HIRESOFS` | DISPLAY | `Encoder` resolver init (`src/inc_encoder.cpp`) — measured resolver channel offset is outside the expected bipolar range. | Resolver excitation/wiring, `sincosofs`, resolver hardware fault. |
| `LORESAMP` | DISPLAY | `Encoder::GetAngleSPI`/resolver angle path — sin/cos signal amplitude hasn't reached `MIN_RES_AMP` yet. | Resolver excitation signal, resolver wiring, whether the motor has resolver hardware at all vs. `encmode` mismatch. |
| `TMPMMAX` | DERATE | `VehicleControl::ProcessThrottle` — motor temperature `tmpm` exceeds `tmpmmax`. | Motor cooling, `tmpmmax` setting, motor temp sensor (`snsm`) calibration. |
| `CANCRC` | STOP | CAN input frame CRC check fails (`src/vehiclecontrol.cpp`, checksum/counter-validated CAN frame handling). | Sender's CRC implementation, bus noise/wiring, frame format mismatch. |
| `CANCOUNTER` | STOP | Same code path — the frame's two sequence-counter copies disagree, or the counter hasn't advanced since the last message. | Sender's sequence-counter implementation, dropped/duplicated frames, bus load. |

## Common bring-up failures

### Motor won't start

`opmode` only leaves `MOD_OFF` inside `Ms10Task()` (`src/stm32_sine.cpp`)
when **all** of these hold (see the `STAT_*` flags built in that function):

- emergency-stop input inactive
- motor-protection input inactive
- throttle not pressed (`STAT_POTPRESSED` clear — check throttle idle
  calibration if this won't clear)
- DC bus voltage ≥ `udcsw`
- DC bus voltage < `udclim`
- a brake pedal has been seen at least once since boot (`seenBrakePedal` —
  this latches on first sight of `din_brake`, or immediately if
  `cruisemode` is `CRUISE_OFF`/`CRUISE_POT`)

...and then only if `din_start` (or `manualstart`, or `tripmode ==
TRIP_AUTORESUME` after a prior trip) is asserted. If the motor won't spin,
read `status` via `get status` or check the fault log with `errors` first
— one of the `STAT_*` conditions above is almost always the reason, not a
control-law problem.

### No throttle response

Throttle path is `VehicleControl::GetUserThrottleCommand` →
`Throttle::CheckAndLimitRange` / `DigitsToPercent` →
`Throttle::CalcThrottle` (`src/throttle.cpp`). Check, in order:

1. **`potmin`/`potmax` (and `pot2min`/`pot2max`) calibration.** If the
   live pot reading falls outside `[potmin, potmax]` by more than a small
   slack (`POT_SLACK`, 200 digits), `CheckAndLimitRange` returns `false`,
   `THROTTLE1`/`THROTTLE2` is posted, and (depending on `potmode`)
   movement is inhibited entirely.
2. **`potmode`.** Determines whether throttle comes from analog input,
   CAN, dual-channel-with-cross-check, or bidirectional mode — each has
   different failure behavior on an out-of-range reading. A CAN-sourced
   throttle (`POTMODE_CAN`) that hasn't received a value in 500 ms also
   returns 0 and posts `CANTIMEOUT`.
3. **Ramp rate.** `throttleRamp` is set from `throtramp` (or the
   parameter's max value below `throtramprpm`) each cycle in
   `ProcessThrottle`; a very low ramp value can look like "no response"
   when it's actually a slow ramp.
4. **Downstream limiters.** Even with a valid throttle, `UdcLimitCommand`,
   `IdcLimitCommand`, `FrequencyLimitCommand`,
   `AccelerationLimitCommand`, and the two `TemperatureDerate` calls can
   all pull the final setpoint back to zero — check `TMPHSMAX`/`TMPMMAX`
   in the error log and the `udc`/`idc`/`fstat` limit parameters.

### Encoder direction/ppr wrong

- Wrong `numimp` (pulses per revolution) makes the reported speed and
  synthesized angle wrong by a scale factor without any fault being
  raised — there's no code-side sanity check on this value against actual
  hardware.
- Wrong `encmode` (encoder type selection) or `respolepairs` (resolver
  pole pairs, only meaningful in resolver mode) produces the same kind of
  silent angle-scaling error. `respolepairs` defaults to 1; verify it
  against your motor before first spin.
- A genuinely noisy or disconnected encoder signal shows up as `ENCODER`
  (pulse-time spike ratio), `HIRESOFS` (resolver offset calibration out of
  range), or `LORESAMP` (resolver signal amplitude too low) in the error
  log — see the fault table above.
- Wrong direction sense (motor spins the wrong way relative to
  forward/reverse input) is a wiring/phase-order or `dirmode` parameter
  question, not something flagged as an error code in this codebase.

### `defaults` / `load` terminal behavior

- `defaults` and `load` are both blocked while the motor is running
  (`opmode != MOD_OFF`) — you'll see "Will not load defaults in run modes,
  please stop before loading!" (or the equivalent for `load`). Run `stop`
  first.
- **This fork fixed a bug where `defaults` didn't actually apply the
  values it loaded** (F18 in [`../FORK_NOTES.md`](https://github.com/moltenmosfet/stm32-sine/blob/fixes/FORK_NOTES.md)): the
  terminal `defaults` command now calls `Param::Change(PARAM_LAST)` after
  resetting values, mirroring what `load`/the SDO path already did. On
  unpatched upstream firmware, `defaults` silently has no effect until an
  unrelated parameter write happens to trigger recalculation.

## Fork-specific notes

Nothing in this fork is hardware-validated yet. Every fix listed in
[`../FORK_NOTES.md`](https://github.com/moltenmosfet/stm32-sine/blob/fixes/FORK_NOTES.md) is host-tested (`cd test && make &&
./test_sine`) and compile-checked for both `CONTROL=SINE` and
`CONTROL=FOC`, but none of it has run on a real board — the dyno hardware
this fork was built for is still being assembled. Treat behavior around
the changed areas (low-speed FOC q-axis clamp, dead-time compensation,
CAN filter setup, scheduler overrun recovery, encoder angle interpolation)
as unverified in practice until that bench pass happens.

## Still stuck?

If a symptom doesn't map to anything above, it's likely a
hardware-wiring, tuning, or board-revision question this repo's code
doesn't answer on its own:

- https://openinverter.org/docs — hardware wiring and setup guide.
- https://openinverter.org/forum — community support and prior-art on
  bring-up issues.
