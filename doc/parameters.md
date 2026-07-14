# Parameter reference

This is a reference for every user-settable parameter and read-only spot
value exposed by this firmware. It is generated from
[`include/param_prj.h`](https://github.com/moltenmosfet/stm32-sine/blob/fixes/include/param_prj.h) at commit `8052200` on the
`fixes` branch (this fork of jsphuebner/stm32-sine), and reflects that file
exactly — if you add or change a parameter, regenerate this doc.

## What parameters are

Each entry in `param_prj.h` defines one tunable value: a category, a name,
a unit, a min/max range, a factory default, and a CAN/SDO id used to
read or write it remotely. Parameters marked "TESTP" in the source are
volatile test/telemetry hooks, not meant to be saved.

## Changing parameters

- **Web interface**: connect over the built-in web UI (via the ESP8266/ESP32
  Wi-Fi bridge or USB) and edit values in the parameter list.
- **Terminal**: over the USB/UART serial console, `set <name> <value>` writes
  a parameter, `get <name>` reads it back, `json` dumps everything as JSON,
  `save` writes the current parameter set to flash, `load` reloads it, and
  `defaults` resets to factory defaults.
- **SDO (CAN)**: read/write by the numeric id in the tables below, using the
  CANopen-style SDO commands (see `libopeninv/src/sdocommands.cpp` and
  `invertersdo.cpp`).

Changes are live in RAM immediately; they are only persisted across a power
cycle after `save` (terminal) or the equivalent SDO/web-UI save action.

## Build variants

This firmware is compiled either `CONTROL=SINE` (V/f induction control) or
`CONTROL=FOC` (field-oriented control), selected at build time
(`include/param_prj.h` lines 236–273). Every table below is one of:

- **both** — present in both builds
- **SINE-only** — only in `CONTROL=SINE` firmware
- **FOC-only** — only in `CONTROL=FOC` firmware

For anything not covered here — wiring, hardware revisions, general tuning
methodology — see the upstream guide: <https://openinverter.org/docs>.

---

## Motor (`CAT_MOTOR`)

### Common to both builds

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| polepairs | – | 1 | 16 | 2 | Motor electrical pole-pair count. Combined with `respolepairs` (ratio `polepairs/respolepairs`) to convert between electrical and mechanical speed, and used in the cruise-control max-speed calculation. |
| respolepairs | – | 1 | 16 | 1 | Resolver/encoder pole-pair count. Divides into `polepairs` to get the electrical-to-mechanical ratio used for speed (rpm) and full-turn counting. |
| sincosofs | dig | 1 | 4096 | 2048 | Zero-angle offset for a SinCos resolver/encoder. |
| encmode | (enum) | 0 | 5 | 0 | Position sensor type/protocol: 0=Single, 1=AB, 2=ABZ, 3=SPI, 4=Resolver, 5=SinCos. |
| fmax | Hz | 21 | 1000 | 200 | Maximum stator/output frequency. Used for the cruise-control max-speed calculation and, in FOC, as the basis of a 110%-of-`fmax` field-weakening-current derate threshold. |
| numimp | ppr | 8 | 8192 | 60 | Encoder pulses per revolution for incremental (AB/ABZ) encoders; selects the encoder timer's capture configuration. |
| dirchrpm | rpm | 0 | 20000 | 100 | Speed below which the direction input is allowed to change the running direction. Above this speed the current direction is held. |
| dirmode | (enum) | 0 | 4 | 1 | Direction input scheme: 0=Button, 1=Switch, 2=ButtonReversed, 3=SwitchReversed, 4=DefaultForward. |
| snsm | (enum) | 12 | 23 | 12 | Motor temperature sensor type, used to convert the raw ADC reading to °C. |

### SINE-only

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| boost | dig | 0 | 37813 | 1700 | 0 Hz voltage boost added to overcome winding resistance at standstill. Reduced automatically as bus voltage rises above `udcnom`. |
| fweak | Hz | 0 | 1000 | 90 | Field-weakening stator frequency reference at full throttle (pot > ~36%); the high end of the throttle-to-frequency map that starts at `fweakstrt`. |
| fweakstrt | Hz | 0 | 1000 | 400 | Field-weakening start frequency; the low end of the throttle-to-frequency map, and the fixed value used at lower throttle. Shifted upward if bus voltage rises above `udcnom`. |
| fconst | Hz | 0 | 1000 | 180 | Stator frequency above which slip is ramped further from `fslipmax` toward `fslipconstmax`. |
| udcnom | V | 0 | 1000 | 0 | Nominal DC bus voltage. When set (>0), deviation of the actual bus voltage from this value trims `fweak` up and `boost` down proportionally, to hold roughly constant motor voltage as the bus sags or rises. 0 disables the compensation. |
| fslipmin | Hz | 0.3 | 10 | 1 | Minimum slip frequency, commanded at low torque. |
| fslipmax | Hz | 0.3 | 10 | 3 | Slip frequency commanded at full torque below `fconst`; ramps further toward `fslipconstmax` above `fweak`. |
| fslipconstmax | Hz | 0 | 10 | 5 | Upper bound on slip frequency once stator frequency exceeds `fconst` (deep field-weakening). |

### FOC-only

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| iqkp | – | 0 | 20000 | 32 | Proportional gain of the q-axis (torque) current PI controller. |
| idkp | – | 0 | 20000 | 32 | Proportional gain of the d-axis (flux) current PI controller. |
| curki | – | 0 | 100000 | 20000 | Integral gain shared by the q-axis and d-axis current PI controllers. |
| exckp | – | 0 | 20000 | 3000 | Proportional gain of the exciter-current PI controller (wound-rotor/separately-excited motors). |
| cogkp | – | -1000 | 1000 | 0 | Anti-cogging feedforward gain, scales the cogging-current estimate into an id/iq correction. 0 = feature off. |
| cogph | – | 0 | 65535 | 0 | Phase offset applied to the anti-cogging injection angle. |
| cogmax | – | 0 | 30000 | 0 | Clamp on the anti-cogging correction magnitude (current digits). |
| vlimflt | – | 0 | 16 | 10 | IIR filter time constant for the field-weakening voltage-limit error signal. |
| vlimmargin | dig | 0 | 10000 | 2500 | Margin subtracted from the maximum modulation index before comparing to actual modulation amplitude, to decide when field-weakening current begins ramping in. |
| fwcurmax | A | -1000 | 0 | -100 | Maximum negative field-weakening (d-axis) current allowed as voltage margin runs out. |
| excurmax | A | 0 | 10 | 0 | Maximum exciter current for wound-rotor/separately-excited motors. |
| syncofs | dig | 0 | 65535 | 0 | Angle offset applied when reading rotor angle for synchronous (encoder-based) operation. |
| lqminusld | mH | 0 | 1000 | 0 | Inductance difference Lq−Ld, used with `fluxlinkage` for the MTPA (maximum-torque-per-amp) calculation on salient machines. |
| fluxlinkage | mWeber | 0 | 1000 | 90 | Rotor permanent-magnet flux linkage, used with `lqminusld` for the MTPA calculation. |
| syncadv | dig/Hz | 0 | 65535 | 10 | Timing advance added to the commanded electrical angle, scaled by filtered stator frequency, to compensate control-loop/sampling delay at speed. |
| qlimfrq | Hz | 0 | 100 | 30 | **Added in this fork.** Frequency below which the low-speed q-axis voltage clamp uses the fork's hysteresis + slew-limited behavior (fixes F1: below ~450 rpm stock code switched clamp polarity instantly with no hysteresis, killing braking/regen and spiking current). `qlimfrq = 0` disables the low-speed clamp policy entirely — used as a deliberate "dyno mode" in this fork's testing. Default 30 approximates stock-equivalent protection. See `FORK_NOTES.md` F1/T4. |
| dtcomp | dig | 0 | 2000 | 0 | **Added in this fork.** Dead-time compensation magnitude added to (or subtracted from, by current sign) each phase's PWM duty in `InvParkClarke`, before short-pulse suppression (fixes F10: switching dead-time was never compensated, distorting control at low torque/current). Default 0 = off, stock behavior. See `FORK_NOTES.md` F10/T5. |

**Hints:**
- `polepairs`/`respolepairs` must divide evenly — the pole-pair ratio is expected to be an integer (`FORK_NOTES.md` F9). A wrong `respolepairs` scales every speed/turns reading; this was flagged as the #1 first-spin trap on one bring-up (`FORK_NOTES.md` T14: default `respolepairs=1` when the actual sensor needs a different value gives a 4x angle error).
- `encmode`/`polepairs`/`respolepairs` are reloaded by a CAN LOAD or terminal `load`; this fork blocks LOAD/`load`/`save` while the motor is running specifically because reloading these mid-run can desync the control loop.
- FOC only: `lqminusld`/`fluxlinkage` (MTPA gain) default to a near-off state and must be set for a salient machine, or MTPA has no effect (`FORK_NOTES.md` F9).
- FOC only: if you enable anti-cogging (`cogkp` ≠ 0), a mis-tuned `cogph` can make the feedforward self-reinforce up to the `cogmax` clamp instead of converging, because the current disturbance the estimator measures includes the injection's own effect (`FORK_NOTES.md` F5, see `include/anticog.h`).
- FOC only: `qlimfrq` and `dtcomp` are new in this fork and both default to values that preserve (`dtcomp`) or approximate (`qlimfrq`) stock behavior — see `FORK_NOTES.md` for the bugs they fix.

---

## Inverter (`CAT_INVERTER`)

### Common to both builds

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| pwmfrq | (enum) | 0 | 2 | 1 | PWM switching frequency: 0=17.6kHz, 1=8.8kHz, 2=4.4kHz. Lower frequencies also raise internal PWM resolution. |
| pwmpol | (enum) | 0 | 1 | 0 | PWM output polarity: 0=ActiveHigh, 1=ActiveLow. Also applied to the boot-loader pin init. |
| deadtime | dig | 0 | 255 | 63 | Raw value written to the STM32 timer's DTG deadtime-generator register — not a linear time value. The DTG encoding is nonlinear above 127 (step size jumps from ~14 ns/count to 111+ ns/count); a value assumed to scale linearly from the sub-127 range yields roughly 2–4x the intended deadtime (`FORK_NOTES.md` F24). |
| ocurlim | A | -65536 | 65536 | 100 | Overcurrent trip threshold, applied symmetrically around the averaged current-sensor offset. |
| il1gain | dig/A | -100 | 100 | 4.7 | Phase 1 current sensor gain. |
| il2gain | dig/A | -100 | 100 | 4.7 | Phase 2 current sensor gain. |
| udcgain | dig/V | -100 | 100 | 6.175 | DC bus voltage sensor gain. |
| udcofs | dig | 0 | 4095 | 0 | DC bus voltage sensor zero offset. |
| udclim | V | 0 | 1000 | 540 | DC bus overvoltage trip threshold. Crossing it sets the `STAT_UDCLIM` status bit and, if the motor isn't stationary, blocks running. |
| snshs | (enum) | 0 | 7 | 0 | Heatsink temperature sensor type: 0=JCurve, 1=Semikron, 2=MBB600, 3=KTY81, 4=PT1000, 5=NTCK45_2k2, 6=Leaf, 7=BMW-i3. |
| pinswap | (bitmask) | 0 | 15 | 0 | Swaps current sensor channels (1), resolver sin/cos lines (2), PWM output pair 1↔3 (4), or PWM output pair 2↔3 (8) in firmware, without rewiring. Bits combine. |

### FOC-only

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| modmax | dig | 37000 | 45000 | 37836 | Maximum modulation index clamping the FOC voltage vector. Lowering it while running is re-applied live (this fork re-clamps the d/q controllers immediately — see hint below). |

**Hints:**
- `ocurlim` must stay ≥ 0 except the Prius-only `-1` sentinel (which disables the hardware break input entirely, handled as a special case). Any other negative value silently inverts the trip window on non-Prius hardware instead of disabling the limit — it's one typo away given the parameter's `-65536` minimum (`FORK_NOTES.md` F24).
- `il1gain` and `il2gain` are averaged together for the shared overcurrent comparator (`ocurlim`) — mismatched sensor gains between the two channels bias that shared threshold rather than being applied independently (`FORK_NOTES.md` F9).
- FOC only: lowering `modmax` at runtime used to be able to make an internal (unsigned) limit calculation underflow and produce a bogus, oversized limit (`FORK_NOTES.md` F6). This fork re-applies the d/q voltage clamp immediately whenever `modmax` changes while running, rather than leaving a stale clamp until the next start.
- `deadtime` and `pwmpol` are only (re)applied to the PWM timer at `PwmInit` (motor start) — see `SetOpmode`/`TimerSetup` in `pwmgeneration.cpp`.

---

## Derating (`CAT_DERATE`)

### Common to both builds

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| bmslimhigh | % | 0 | 100 | 50 | Percent scaling applied to positive (motoring) torque commands — e.g. from a BMS charge-current limit. |
| bmslimlow | % | -100 | 0 | -1 | Percent scaling applied to negative (regen/braking) torque commands. |
| udcmin | V | 0 | 1000 | 450 | DC bus voltage below which undervoltage torque derating begins (a small internal margin is added). |
| udcmax | V | 0 | 1000 | 520 | DC bus voltage above which overvoltage torque derating begins (a small internal margin is added). |
| idcmax | A | 0 | 5000 | 5000 | DC current above which current-based (motoring) derating begins. |
| idcmin | A | -5000 | 0 | -5000 | DC current below (more negative than) which current-based (regen) derating begins. |
| idckp | dig | 0.1 | 20 | 2 | Proportional gain of the DC current derating controller. |
| idcflt | dig | 0 | 11 | 9 | IIR filter time constant for the measured DC current feeding the current derating loop. |
| tmphsmax | °C | 50 | 150 | 85 | Heatsink temperature above which torque is derated. |
| tmpmmax | °C | 70 | 300 | 300 | Motor temperature above which torque is derated. |
| throtmax | % | 0 | 100 | 100 | Upper clamp on the final throttle command. |
| throtmin | % | -100 | 0 | -100 | Lower clamp (regen limit) on the final throttle command. |
| accelmax | rpm/10ms | 1 | 1000 | 1000 | Maximum allowed speed change per 10 ms — an acceleration/deceleration rate limiter. |
| accelflt | dig | 1 | 5 | 3 | IIR filter time constant applied to the measured speed delta feeding the acceleration limiter. |

### SINE-only

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| iacmax | A | 0 | 5000 | 5000 | Maximum AC (motor phase) current before current-limiting reduces the commanded amplitude. 0 disables current limiting. |
| ifltrise | dig | 0 | 32 | 10 | IIR filter time constant for the current-limit setpoint while it is rising (recovering headroom). |
| ifltfall | dig | 0 | 32 | 3 | IIR filter time constant for the current-limit setpoint while it is falling (clamping harder) — faster by default than `ifltrise`. |

**Hints:**
- `throtmax`/`throtmin`, `idcmax`/`idcmin`, and `offthrotregen` (Regen category) are handled as a fast-path in the parameter-change dispatcher because they're expected to be written frequently over CAN (see `stm32_sine.cpp`).
- `bmslimhigh`/`bmslimlow` and the manual FOC test currents (`manualiq`/`manualid`, Testing category) are separate derate paths — the manual test currents bypass the normal throttle-path derates entirely (`FORK_NOTES.md` F9). Don't leave a manual test current set when handing the vehicle back to normal throttle control.

---

## Charger (`CAT_CHARGER`)

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| chargemode | (enum) | 0 | 4 | 0 | Charger operating mode, repurposing the inverter as an on-board charger: 0=Off, 3=Boost, 4=Buck. |
| chargecur | A | 0 | 500 | 0 | Target charge current fed to the charger current controller. |
| chargekp | dig | -100 | 100 | 80 | Proportional gain of the charger current PI controller. |
| chargeki | dig | -100 | 100 | 10 | Integral gain of the charger current PI controller. |
| chargeflt | dig | 0 | 10 | 8 | IIR filter time constant on the measured charge current. |
| chargepwmin | % | 0 | 99 | 0 | Minimum PWM duty clamp for the charger controller output. |
| chargepwmax | % | 0 | 99 | 90 | Maximum PWM duty clamp for the charger controller output. |

**Hints:**
- Which precharge voltage threshold applies depends on `chargemode`: Buck mode precharges toward `udcswbuck` (Contactor Control category), Boost mode toward `udcsw`.

---

## Throttle (`CAT_THROTTLE`)

### Common to both builds

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| potmin | dig | 0 | 3500 | 0 | Raw ADC value at throttle pot channel 1's released/minimum position. Used for pot range-fault checking and percent scaling. |
| potmax | dig | 0 | 3500 | 3500 | Raw ADC value at throttle pot channel 1's full-throttle position. |
| pot2min | dig | 0 | 4095 | 4095 | Raw ADC value, minimum position of throttle pot channel 2 (dual-channel or brake-pot use). |
| pot2max | dig | 0 | 4095 | 4095 | Raw ADC value, maximum position of throttle pot channel 2. |
| potmode | (bitmask/enum) | 0 | 6 | 0 | Throttle input scheme: 0=SingleRegen (regen-adjust dial), 1=DualChannel, 2=CAN, 3=CANDual, 4=BiDir, 6=CANBiDir. |
| potlinearity | % | 0 | 100 | 100 | Percent shaping factor applied to the scaled throttle percentage before it becomes the torque command. |
| throtramp | %/10ms | 0.1 | 100 | 100 | Throttle ramp rate, applied below `throtramprpm`. |
| throtramprpm | rpm | 0 | 20000 | 20000 | Speed above which the `throtramp` ramp-rate limit is no longer applied (ramping becomes effectively instant). |

### SINE-only

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| ampmin | % | 0 | 100 | 10 | Minimum output amplitude at zero torque request — the floor the voltage curve scales up from. |
| slipstart | % | 10 | 100 | 50 | Throttle percentage at which the "sequential" (VoltageSlip) curve switches from ramping amplitude only to also ramping slip. |
| sinecurve | (enum) | 0 | 1 | 0 | Throttle-to-output curve shape: 0=VoltageSlip (amplitude first, then slip), 1=Simultaneous (both ramp together). |
| throtfilter | dig | 0 | 10 | 4 | IIR filter time constant (shift count) applied to amplitude/slip ramping. |

### FOC-only

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| throtcur | A/% | 0 | 10 | 1 | Torque-current scaling: throttle percent × this value becomes the stator current magnitude setpoint fed into MTPA. |

**Hints:**
- `potmode` is a bitmask on top of being a small enum: `POTMODE_BIDIR` (4) and `POTMODE_CAN` (2) can combine with `POTMODE_DUALCHANNEL` (1), which is why the valid range runs to 6 rather than a dense 0–4.
- When `potmode` has the BiDir bit set, direction is determined by the throttle pot itself, not the direction input (`dirmode` is skipped in that case).

---

## Regen (`CAT_REGEN`)

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| brakeregen | % | -100 | 0 | -50 | Regen percentage commanded when the brake pedal (digital input) is pressed. |
| regenramp | %/10ms | 0.1 | 100 | 100 | Ramp rate applied to regen commands. |
| regentravel | % | 0 | 100 | 30 | Percentage of pot travel over which regen ramps from `brakeregen` toward zero as pot 2 (used as a brake transducer) is released. Dynamically rescaled with speed if `maxregentravelhz` is nonzero. |
| offthrotregen | % | -100 | 0 | -30 | Regen percentage applied when the throttle is fully released (off-throttle). |
| cruiseregen | % | -100 | 0 | -30 | Maximum regen percentage the cruise-control speed loop is allowed to command. |
| regenrampstr | Hz | 0 | 400 | 10 | Rotor frequency below which brake-regen ramping engages, for near-standstill handling. |
| maxregentravelhz | Hz | 0 | 1000 | 0 | Rotor frequency at which the dynamic `regentravel` scaling reaches its maximum. 0 disables the speed-based scaling — `regentravel` is then used directly at all speeds. |
| brklightout | % | -100 | -1 | -50 | Torque threshold (negative) below which the brake-light output is driven, with a 2-point hysteresis band on release. |

**Hints:**
- `offthrotregen` is auto-corrected to negative on parameter load if a config carries a stale positive value (a one-time "upgrade parameter" fixup in `stm32_sine.cpp`) — you shouldn't need to set it negative by hand after that, but the valid range only accepts ≤0 going forward.
- `regenrampstr` and near-standstill regen behavior interact with the encoder's frequency deadband: rotor frequency reads exactly 0 below roughly 42 rpm regardless of actual rotation, which affects any speed-gated regen behavior below that threshold (`FORK_NOTES.md` F4).
- `maxregentravelhz = 0` is a deliberate off-switch for the dynamic (speed-based) part of `regentravel`, not a typo to avoid.

---

## Automation (`CAT_AUTOM`)

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| idlespeed | rpm | -100 | 10000 | -100 | Target idle/creep speed for the idle-speed proportional controller. |
| idlethrotlim | % | 0 | 100 | 50 | Maximum throttle percentage the idle-speed controller may command. |
| idlemode | (enum) | 0 | 4 | 3 | When idle/creep torque is applied: 0=Always, 1=NoBrake, 2=Cruise, 3=Off, 4=HillHold. |
| holdkp | – | -100 | 0 | -0.25 | Proportional gain (negative) of the hill-hold controller — converts rollback distance into a holding torque command. |
| speedkp | – | 0 | 100 | 0.25 | Proportional gain shared by the cruise-control and idle-speed loops. |
| speedflt | – | 0 | 16 | 5 | IIR filter time constant applied to measured speed for the cruise-control speed error. |
| cruisemode | (enum) | 0 | 4 | 0 | Cruise-control source: 0=Off, 1=Switch, 2=CAN, 3=ThrottlePot, 4=Limiter. |
| cruisethrotlim | % | 0 | 100 | 50 | Maximum throttle percentage the cruise-control speed loop may command. |

**Hints:**
- When `idlemode=Always` and `potmode=SingleRegen` (0), `idlethrotlim` is dynamically reduced as pot 2 (brake pressure) is released, on top of its configured value.
- `cruisemode=Limiter` behaves differently from the other modes: it never commands acceleration on its own, so it's left running continuously and only clamps throttle down to the cruise speed rather than replacing the throttle command.

---

## Contactor Control (`CAT_CONTACT`)

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| udcsw | V | 0 | 1000 | 330 | DC bus voltage above which the bus is considered charged enough to run. Below half this value, past the precharge timeout with the precharge output active, a fault is raised. Also gates `STAT_UDCBELOWUDCSW`. |
| udcswbuck | V | 0 | 1000 | 540 | DC bus voltage threshold used for the precharge target when `chargemode=Buck` (vs. `udcsw` used for Boost). |
| tripmode | (enum) | 0 | 3 | 0 | Behavior after a trip/fault: 0=AllOff, 1=DcSwOn, 2=PrechargeOn, 3=AutoResume (lets the inverter resume RUN without a fresh start command, same path as `manualstart`). |
| bootprec | On/Off | 0 | 1 | 0 | Whether the precharge output is asserted from the bootloader's pin-init state, before firmware fully starts. |
| outmode | (enum) | 0 | 2 | 0 | What drives the auxiliary output normally used for the DC contactor: 0=DcSw (contactor control), 1=TmpmThresh, 2=TmphsThresh. |
| fanthresh | °C | 20 | 300 | 50 | Temperature above which the cooling fan output switches on, with a 5 °C hysteresis band on release. |

**Hints:**
- `outmode` = TmpmThresh or TmphsThresh repurposes the auxiliary output as a threshold-driven signal instead of contactor control — in the FOC build specifically, both of these non-default modes were found to be nominally selectable but not actually wired to drive anything (`FORK_NOTES.md` F24). Leave `outmode=DcSw` on FOC unless you've verified otherwise for your hardware revision.
- `tripmode=AutoResume` and the Testing category's `manualstart` share the same RUN-restart path in `stm32_sine.cpp`; a remote CAN start via this path bypasses the normal start interlocks (`FORK_NOTES.md` F11) — sequencing safety is the caller's responsibility.

---

## Aux PWM (`CAT_PWM`)

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| pwmfunc | (enum) | 0 | 3 | 0 | What quantity the auxiliary PWM output tracks: 0=tmpm, 1=tmphs, 2=speed, 3=exciter. |
| pwmgain | – | -100000 | 100000 | 100 | Linear gain applied to the selected `pwmfunc` quantity to produce the PWM output value. |
| pwmofs | dig | -65535 | 65535 | 0 | Linear offset added to the scaled `pwmfunc` quantity. |

---

## Communication (`CAT_COMM`)

| Name | Unit | Min | Max | Default | Description |
|---|---|---|---|---|---|
| canspeed | (enum) | 0 | 4 | 2 | CAN bus baud rate: 0=125k, 1=250k, 2=500k, 3=800k, 4=1M. |
| canperiod | (enum) | 0 | 1 | 0 | Periodic CAN message map send rate: 0=100ms, 1=10ms. |
| nodeid | – | 1 | 63 | 1 | CANopen/SDO node ID used for parameter access over CAN. |
| controlid | – | 1 | 2047 | 63 | CAN identifier the vehicle-control throttle/command message is expected on. |
| controlcheck | (enum) | 0 | 1 | 1 | Validation applied to the incoming control CAN message: 0=CounterOnly, 1=StmCrc8. |

**Hints:**
- The CAN receive filter setup (which frames the controller actually accepts, independent of `controlid`) had three bugs fixed in this fork — a leftover-filter flush using the wrong index, mask-bank checks against the wrong per-bank count, and an incompletely filled bank silently accepting CAN ID 0 (`FORK_NOTES.md` F12). Not a parameter, but relevant if CAN behavior looks off after changing `nodeid`/`controlid`.

---

## Testing (`CAT_TEST`)

| Name | Unit | Min | Max | Default | Build | Description |
|---|---|---|---|---|---|---|
| manualstart | On/Off | 0 | 1 | 0 | both | One-shot flag that forces `opmode` to RUN, self-clearing after use. Lets you start the inverter from the terminal/CAN without a hardware start input. Shares the `tripmode=AutoResume` restart path, which bypasses normal start interlocks. |
| fslipspnt | Hz | -100 | 1000 | 0 | SINE-only | Test hook: writes the slip-frequency setpoint directly, bypassing the throttle-to-slip calculation. For bench testing. |
| ampnom | % | 0 | 100 | 0 | SINE-only | Test hook: writes the output amplitude setpoint directly, bypassing the throttle-to-amplitude calculation. For bench testing. |
| manualiq | A | -400 | 400 | 0 | FOC-only | Test hook: adds a directly-commanded q-axis (torque) current on top of the throttle-derived MTPA reference. |
| manualid | A | -400 | 400 | 0 | FOC-only | Test hook: adds a directly-commanded d-axis (flux) current on top of the MTPA reference. |

**Hints:**
- `manualiq`/`manualid` bypass the normal throttle-path safety derates entirely (`FORK_NOTES.md` F9) — they're for bench/dyno current-loop testing, not for driving. Zero them before handing control back to the throttle.
- The terminal `start` command (distinct from `manualstart` the parameter) used to run a fixed-point conversion on a plain integer, which shifted most mode values down to "off" and made the command silently do nothing for most inputs — fixed in this fork (`FORK_NOTES.md` F19).

---

## Read-only spot values

These are telemetry/status values, not settable parameters. They can still
be read over the terminal (`get <name>`) or by SDO id, and mapped onto
periodic CAN messages, but writes have no lasting effect (some are written
internally by the firmware every control cycle).

| Name | Unit | Description |
|---|---|---|
| version | (string) | Firmware version string. |
| hwver | (enum) | Detected hardware revision. |
| opmode | (enum) | Current operating mode: Off/Run/ManualRun/Boost/Buck/Sine/AcHeat. |
| lasterr | (string) | Last posted error message. |
| status | (bitmask) | Current status/fault flags (UdcLow, UdcHigh, UdcBelowUdcSw, UdcLim, EmcyStop, MProt, PotPressed, TmpHs, WaitStart, BrakeCheck). |
| udc | V | Measured DC bus voltage. |
| idc | A | Estimated DC bus current. |
| il1 | A | Measured phase 1 current. |
| il2 | A | Measured phase 2 current. |
| fstat | Hz | Filtered stator frequency. |
| speed | rpm | Motor speed. |
| cruisespeed | rpm | Active cruise-control speed setpoint; -1 when cruise is inactive. |
| turns | – | Accumulated full motor turns (from the encoder pole counter). |
| amp | dig | Commanded output amplitude. |
| angle | ° | Current electrical angle. |
| pot | dig | Raw throttle pot 1 reading. |
| pot2 | dig | Raw throttle pot 2 reading. |
| regenpreset | % | Regen-adjust dial position, when `potmode=SingleRegen`. |
| potnom | % | Scaled/final throttle command. |
| seldir | (enum) | User-selected direction. |
| rotordir | (enum) | Actual rotor direction of rotation. |
| tmphs | °C | Measured heatsink temperature. |
| tmpm | °C | Measured motor temperature. |
| uaux | V | Auxiliary supply voltage. |
| pwmio | – | Raw PWM I/O pin configuration readback. |
| canio | (bitmask) | Digital I/O states mirrored over CAN (Cruise/Start/Brake/Fwd/Rev/Bms). |
| din_cruise | On/Off | Cruise-control digital input state. |
| din_start | On/Off | Start digital input state. |
| din_brake | On/Off | Brake digital input state. |
| din_mprot | Ok/Error | Motor-protection switch input state. |
| din_forward | On/Off | Forward direction digital input state. |
| din_reverse | On/Off | Reverse direction digital input state. |
| din_emcystop | Ok/Error | Emergency-stop digital input state. |
| din_ocur | Ok/Error | Hardware overcurrent comparator input state. |
| din_desat | Ok/Error | IGBT/MOSFET desaturation fault input state. |
| din_bms | On/Off | BMS digital input state. |
| dout_brake | On/Off | Brake-light output state. |
| uptime | 10ms | Time since boot. |
| cpuload | % | CPU load. |

### SINE-only

| Name | Unit | Description |
|---|---|---|
| ilmax | A | Larger of the two measured phase RMS currents. |
| uac | V | Estimated AC output (motor terminal) voltage. |
| il1rms | A | Phase 1 RMS current. |
| il2rms | A | Phase 2 RMS current. |
| boostcalc | dig | Effective 0 Hz boost after `udcnom`-based trim. |
| fweakcalc | Hz | Effective field-weakening frequency after `udcnom`-based trim. |

### FOC-only

| Name | Unit | Description |
|---|---|---|
| id | A | Measured d-axis (flux) current. |
| iq | A | Measured q-axis (torque) current. |
| ifw | A | Field-weakening/exciter current. |
| ud | dig | Commanded d-axis voltage. |
| uq | dig | Commanded q-axis voltage. |
| uexc | dig | Exciter PWM output. |
| anticog | dig | Current anti-cogging correction output. |

---

## Fork-added parameters

Everything below is new in this fork relative to upstream `jsphuebner/stm32-sine`
(`git diff master -- include/param_prj.h`). Both default to values that
reproduce stock (pre-fork) behavior, so an existing saved parameter set
loads with no behavior change until you deliberately opt in.

| Name | Category | Build | id | Default | What it fixes |
|---|---|---|---|---|---|
| `qlimfrq` | Motor | FOC-only | 165 | 30 (0 = old unclamped/dyno-mode behavior) | F1: below ~450 rpm, the stock q-axis voltage clamp switched polarity instantly with no hysteresis or ramp, killing braking/regen and spiking current at the threshold. |
| `dtcomp` | Motor | FOC-only | 166 | 0 (off) | F10: switching dead-time was never compensated in the modulation, producing a voltage error large enough to distort control at low torque and low current. |

See `FORK_NOTES.md` (F1, F10, and the T4/T5 work items) for the full writeup,
including the glossary of F-numbered findings referenced as hints throughout
this document.
