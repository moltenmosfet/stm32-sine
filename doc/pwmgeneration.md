# PwmGeneration

## Description
`PwmGeneration` (`include/pwmgeneration.h`) is a static class that owns the
PWM timer and turns a torque/throttle request into duty cycles on the three
motor phases. The class body is shared; behavior splits across three files:

- `src/pwmgeneration.cpp` — shared infra: opmode state machine, timer
  setup/teardown (`TimerSetup`), the two ISRs (`tim1_brk_isr`,
  `pwm_timer_isr`), current-offset/overcurrent-threshold calibration, the
  buck/boost charger (`Charge`) and AC-heat (`AcHeat`) modes.
- `src/pwmgeneration-sine.cpp` — `CONTROL=SINE` build (`make`, default):
  open-loop/slip-frequency sine generation via `SineCore`.
- `src/pwmgeneration-foc.cpp` — `CONTROL=FOC` build (`make CONTROL=FOC`):
  closed-loop field-oriented control via `FOC` and `PiController`
  (`libopeninv`).

Only one variant file compiles per image (`Makefile` sets
`-DCONTROL=CTRL_SINE`/`CTRL_FOC`). Public interface (all static): `Run()`
(called every PWM ISR), `SetOpmode`, `SetAmpnom`, `SetFslip`,
`SetTorquePercent` (one impl per variant), `SetCurrentOffset`,
`SetCurrentLimitThreshold`, `SetControllerGains`, `UpdateVoltageLimits`
(FOC only), `GetCpuLoad`, `SetChargeCurrent`, `SetPolePairRatio`,
`SetFwExcCurMax`, `GetAngle`, `Tripped`.

Shared `param_prj.h` params: `pwmfrq`, `pwmpol`, `deadtime`, `ocurlim`,
`il1gain`/`il2gain`, `pinswap`, `iacmax`, `ifltrise`/`ifltfall`, `idcflt`,
`chargepwmin`/`chargepwmax`/`chargekp`/`chargeki`. FOC adds `modmax`,
`vlimmargin`, `vlimflt`, `syncofs`, `syncadv`, `qlimfrq`, `dtcomp`,
`cogkp`/`cogph`/`cogmax`, `manualid`/`manualiq`. SINE adds `ampmin`,
`slipstart`, `sinecurve`, `throtfilter`, `fslipmin`/`fslipmax`,
`fslipconstmax`, `fweakcalc`, `fconst`.

## Why?
PWM generation runs inside the PWM timer's update interrupt and must finish
well inside one PWM period. Splitting shared timer/current-sense plumbing
from the two control strategies lets the same low-level code (timer setup,
overcurrent comparator, deadtime, pin-swap) serve both a simple open-loop
drive and a full current-vector-controlled drive, selected at compile time
instead of carrying both code paths in one binary.

## Drawbacks
- `Run()` in both variants is one long function with static locals mixing
  sensing, control, and output — no internal separation to trace a signal.
- FOC's exciter branch (`hwRev == HW_ZOE`) is inlined into `Run()` with no
  test coverage; easy to break while changing the general path.
- Both variants' current-limit/clamp logic uses ad-hoc IIR filters with
  magic constants (`a = imax/20`, 40% floor, `slewDivisor = 256`) whose
  reasoning lives only in inline comments.
- Timing correctness is implicit: `GetCpuLoad()` reports elapsed ticks but
  nothing asserts if a cycle overruns the PWM period.

## Architecture
`pwm_timer_isr` fires on the PWM timer's update event, clears the pending
flag, calls `Run()`, and measures elapsed ticks into `execTicks`
(`GetCpuLoad()`). Callback frequency is constant regardless of switching
frequency via the timer's repetition counter (`TimerSetup`, `repCounters[]`
picks 4/2/1 update events per callback across the three `pwmfrq` settings).

`tim1_brk_isr` handles the break interrupt (hardware fault: desat, e-stop,
motor-protection, overcurrent depending on `hwRev`), posts the matching
`ErrorMessage`, forces `opmode = MOD_OFF`, sets the sticky `tripped` flag.

`Run()` dispatches on `opmode`: `MOD_MANUAL`/`MOD_RUN`/`MOD_SINE` (SINE) or
`MOD_RUN` (FOC) run motor control; `MOD_BOOST`/`MOD_BUCK` call `Charge()`
(one `PiController` driving the buck/boost charger); `MOD_ACHEAT` calls
`AcHeat()` (low-frequency single-leg resistive heating via the windings).

Fixed-point convention: angles are `uint16_t`, 65536 = one electrical
revolution; currents/voltages are `s32fp` (`my_fp.h`, `libopeninv`).
`FRQ_TO_ANGLE` converts an `s32fp` frequency to a per-cycle angle increment.
Both variants share `GetCurrent()` (ADC → offset-corrected, gain-scaled
current) and `TimerSetup()` (center-aligned PWM, deadtime — a raw STM32 DTG
register code, not linear time; see F24 in `FORK_NOTES.md` — polarity, and
`pinswap` channel remapping via `ocChannels[]`).

**SINE**: `Run()` advances the encoder, computes a current-limited amplitude
(`LimitCurrent()`), advances angle (`CalcNextAngleConstant` for `MOD_SINE`,
`CalcNextAngleAsync` — adds slip on top of measured angle — otherwise),
converts to a voltage percentage (`MotorVoltage::GetAmpPerc`, `libopeninv`),
and writes `SineCore::Calc(angle)`'s duty cycles to the timer.
`SetTorquePercent()` maps torque percent to `ampnom`/`fslip` via a
simultaneous (`sinecurve`) or sequential (default) ramp curve, IIR-filtered
by `throtfilter`. `LimitCurrent()` derates amplitude and slip past 80% of
`iacmax` (independent `ifltrise`/`ifltfall` filters, 40% amplitude floor)
using RMS current from `ProcessCurrents()`/`GetIlMax()` (`fp_hypot3`).

**FOC**: `Run()` (`MOD_RUN` only) computes `id`/`iq` (`FOC::ParkClarke`),
runs a field-weakening loop that derates `iq`/`id` references as voltage
headroom (`vlim`, from `GetMaximumModulationIndex() - vlimmargin - amp`)
shrinks, adds anti-cogging feedforward to the d-axis command, and runs
`dController`/`qController` to produce `ud`/`uq`, converted to duty cycles
by `FOC::InvParkClarke` at a `syncadv`-advanced angle. `hwRev == HW_ZOE`
runs a third controller against measured exciter current instead.
`UpdateVoltageLimits()` re-applies the `±(modmax-1000)` controller range if
`modmax` changes mid-`MOD_RUN` (fork-fixed, F6/T3, `FORK_NOTES.md`) —
without it a lowered `modmax` leaves a stale clamp from RUN entry.
`SetTorquePercent()` derives an MTPA-optimal `id`/`iq` target via
`FOC::Mtpa`; at zero torque/speed/manual overrides, `isIdle` disables PWM
and runs offset recalibration (skipped on `HW_PRIUS`).

Two fork additions here, both default to stock behavior:
- **Low-speed q-axis clamp (F1)** — `QClamp::Update()` replaces an instant
  frequency-threshold quadrant restriction with hysteresis+slew; see
  `doc/qclamp.md`. `qlimfrq=0` (this fork's dyno-mode value) disables the
  restriction; nonzero reproduces stock's restriction without the snap.
- **Dead-time compensation (F10/T5)** — `InvParkClarke`'s `dtcomp` argument
  (param id 166) defaults to 0 (off); nonzero applies a phase-sign voltage
  correction inside `InvParkClarke` (`libopeninv`) countering F10.

Anti-cogging feedforward (`cogkp`/`cogph`/`cogmax`, `AntiCogTriangle` from
`include/anticog.h`) is upstream functionality with a fork-fixed wraparound
bug; see `doc/anticog.md`.

## Stability
Host-tested via `cd test && make && ./test_sine` (`test_getilmax.cpp`
covers the `fp_hypot3` overflow fix) and compile-checked with `make
CONTROL=SINE`/`CONTROL=FOC`. No hardware bring-up on this fork yet — treat
both variants, including every fork-added change above, as unvalidated on
real motors/inverters until a bench pass confirms them.
