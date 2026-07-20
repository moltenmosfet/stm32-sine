# Fork notes — moltenmosfet/stm32-sine (dyno absorber)

This is the Molten MOSFET dyno fork of Open Inverter's `stm32-sine`, driving a
Nissan Leaf EM57 motor as the absorber on a chassis dynamometer. Rationale for
individual changes is in the per-task rows below; deeper design rationale that
isn't captured here lives in internal review notes (not part of this repo).

## Two repos, two forks

`stm32-sine` vendors `libopeninv` as a submodule; each has its own upstream and fork:

| Repo | Upstream (`origin`) | Fork (`fork`) | Baseline pin |
|---|---|---|---|
| superproject | jsphuebner/stm32-sine | moltenmosfet/stm32-sine | `1dfab85` |
| libopeninv    | jsphuebner/libopeninv | moltenmosfet/libopeninv | `78e3f72` |

`libopencm3` submodule is left at its pin (`5ba1bb5`); not forked.

All review line numbers refer to these two pins. `fixes` is the integration branch
in each repo, created off the pin and pushed to `fork`. Task branches (`fix/T<nn>-<slug>`)
fork off `fixes` and merge back after review.

### libopeninv submodule discipline
A task touching `libopeninv/` commits inside `libopeninv/`, pushes that branch to the
libopeninv fork, then bumps the submodule pointer in the superproject branch. Never let
a superproject branch reference a libopeninv SHA that isn't pushed to the fork.

## Upstream PR policy
`origin` stays pointed at upstream in both repos. `[UPSTREAM]` tasks become clean
branches off *current* upstream master at submission time (rebase then; develop against
the pin now). `[FORK]` tasks encode dyno policy and stay on the fork. Never push to
`origin` (jsphuebner) — forks only.

PR sequence: tasks marked UPSTREAM in the table below (T2, T3, T6's race-condition half,
T8, T9, T10, T11, T12, T15, T16, T17, T18, T19) are submitted as separate PRs, library-level
fixes in libopeninv going out before the superproject fixes that depend on them. Tasks
marked FORK (T1, T4, T5, T6's taper half, T7, T13, T14, T20) stay on the fork; T5
(deadtime compensation) may be offered upstream once bench-validated.

## Findings glossary

Each finding below (F1–F24) came out of a source-level review of the FOC control path,
the CAN/SDO stack, the scheduler, and the SINE build; F25 was found while porting the
upstream `35=AutoTune` branch. Task rows in the table further down reference these
numbers.

| # | What the defect is |
|---|---|
| F1 | Below about 450 rpm, the q-axis voltage clamp only allows one polarity, switched instantly with no hysteresis or ramp — kills braking/regen at low speed and spikes current at the threshold. |
| F2 | The PI controller's integral term updates in coarse steps (~1.7% of full modulation) at default gains because an integer division truncates before scaling — causes a low-torque limit cycle. |
| F3 | Phase-current sampling free-runs unsynchronized to the PWM switching, so samples land at random points on the current ripple, adding noise that's significant at partial load. |
| F4 | Below about 42 rpm the frequency estimate is forced to zero and the last direction stays latched — regen torque cuts on/off abruptly and direction can flap during near-standstill oscillation. |
| F5 | The anti-cogging feedforward (off by default) has a 16-bit angle wraparound glitch and an unfiltered amplitude estimate that can self-reinforce; only matters if the feature is turned on. |
| F6 | Lowering the modulation-limit parameter while the motor is running can make an internal limit calculation go negative under unsigned arithmetic, producing a bogus limit and an overmodulation spike. |
| F7 | An unused variant of the PI controller (present in the shared library, not used by this firmware) truncates its output to an integer — would quantize control for any other firmware built on the same library. |
| F8 | Reviewed and cleared: several suspected issues (double application of a timing-advance term, anti-windup interaction with dynamic clamps, overflow risk in voltage math, short-pulse suppression) turned out not to be bugs. |
| F9 | Configuration traps for this motor: pole-pair ratio must be an integer, an MTPA gain is off by default on a salient machine, the regen voltage taper must sit above the resistor-dump voltage, overcurrent thresholds are averaged across both current sensors, and the manual torque test parameters bypass throttle-path safety derates. |
| F10 | Switching dead-time is never compensated in the modulation, producing a voltage error large enough to distort control at low torque and low current. |
| F11 | CAN commands to reload saved parameters or restore defaults aren't blocked while the motor is running — a remote command mid-run can switch encoder mode and scramble the control loop. A separate remote-start path also skips the contactor-close interlock. |
| F12 | Three bugs in the CAN receive filter setup: a leftover-filter flush uses the wrong index (silently drops an extended-ID filter), mask banks are checked against the wrong per-bank count (wastes half of every mask bank), and an incompletely filled bank accidentally accepts CAN ID 0. |
| F13 | A counter shared between the fast PWM interrupt and the slower frequency-update interrupt can be read and cleared non-atomically, occasionally losing an angle increment and undercounting frequency. |
| F14 | Writing most parameters over CAN triggers a full reconfiguration routine inside the CAN receive interrupt, adding tens of microseconds of latency per write — noticeable for high-rate commands like a torque setpoint. |
| F15 | If a scheduled task overruns its period, the 16-bit hardware timer compare can end up behind the counter, and the task then silently stalls for up to roughly 650 ms before it fires again. |
| F16 | Minor items grouped together: a fixed-point filter with a small negative rounding bias, one filter's state shared by two callers running it at twice the intended rate, an ADC-channel-count off-by-one on one hardware variant, a self-correcting startup transient in a cogging estimate, missing length checks on incoming CAN SDO frames, and a start interlock that doesn't count a regen (negative) command as "throttle pressed." |
| F17 | A 2023 fix for current-magnitude overflow above ~1000 A was added to the math library but never wired into the function it was meant to fix — the overflow is still present. |
| F18 | The terminal "defaults" command resets parameter values in memory but never triggers the recalculation that applies them, so the reset has no effect until an unrelated parameter write happens to trigger it. |
| F19 | The terminal "start" command runs a fixed-point conversion on a plain integer, shifting most mode values down to "off" — the command silently does nothing for most inputs. |
| F20 | An RMS calculation divides by a value derived from frequency with no floor, dividing by zero below 1 Hz stator frequency. |
| F21 | A logic condition in sine-wave current processing uses OR where AND was intended, making it always true; currently harmless because that code path isn't reached in the affected mode. |
| F22 | Angle interpolation between pulses on single-channel encoders isn't clamped, so it can overshoot past the next real pulse and then snap back — a sawtooth in the synthesized angle. |
| F23 | A regen safety guard meant to block driving in the wrong direction compares a value against itself in single-channel encoder mode, so it can never trigger — looks like protection, does nothing. |
| F24 | Documentation/config traps for all users: a negative overcurrent-limit parameter silently inverts the trip thresholds on non-Prius hardware instead of disabling the limit; the deadtime parameter is a nonlinear hardware register code, not a linear time value; two output modes stay nominally enabled in the FOC build without actually driving anything; and current-limiting has undocumented floors below which it can't act. |
| F25 | In the upstream `35=AutoTune` branch, the bidirectional test-spin's averaged sync-error output can never publish: the test angle always steps in twos (even values only) but the pass-end check compares against an odd endpoint, so the condition is unreachable and the intermediate half-average is silently overwritten every revolution instead. |

## Applied tasks

Branch naming `fix/T<nn>-<slug>`, merged `--no-ff` into `fixes` in each repo.
Verification: host suite = `cd test && make && ./test_sine`; firmware = `make
CONTROL=FOC` and `make CONTROL=SINE` (ARM toolchain became available mid-wave-2,
so all firmware-only changes are now compile-checked).

| Task | Type | Repo(s) | Verification | Status |
|---|---|---|---|---|
| T0 | FORK | both | n/a | done — forks, remotes, fixes, this file |
| T1 | — | superproject | host suite | done — picontroller/foc under test, libopencm3 stubs |
| T2 | UPSTREAM | libopeninv (+super bump) | host tests (3) + FOC/SINE build | done — F2/F7 |
| T3 | UPSTREAM | libopeninv + superproject | host test + FOC build | done — F6 (GetQLimit host-tested; UpdateVoltageLimits firmware-built) |
| T7 | FORK | superproject | FOC/SINE build + host | done — F14 (guarded #if CTRL_FOC; fast-paths `manualid`/`manualiq` so CAN torque commands skip the reconfig cost) |
| T8 | UPSTREAM | libopeninv (+super bump) | review + FOC build | done — F11 (review-verified; an automated test harness for this path wasn't cost-effective) |
| T15 | UPSTREAM | superproject | host test + SINE/FOC build | done — F17 |
| T17 | UPSTREAM | superproject | FOC/SINE build + host | done — F19/F20/F21 (3 commits) |
| T4 | FORK | superproject | host tests (7) + FOC/SINE build | done — F1 (QClamp policy class, `qlimfrq` param id 165, 0 = dyno mode / restriction off; shrinking qlimit clamps instantly; dead QLIMIT_FREQUENCY macro removed) |
| T5 | FORK→UPSTREAM cand. | libopeninv + superproject | host tests (4) + FOC/SINE build | done — F10 (`dtcomp` param id 166, default 0 = off; signs captured in ParkClarke, applied in InvParkClarke before short-pulse suppression; 2 A deadband; pinswap resolved at call site) |
| T6 | UPSTREAM (race) / FORK (taper) | superproject | host tests (4) + FOC/SINE build | done — F13/F4 (atomic copy+subtract of turnsSinceLastSample under cm_disable_interrupts, both encoder-mode branches; RegenTaperHold holds last taper factor 500 ms across the zero-freq deadband; separate commits for the PR split) |
| T11 | UPSTREAM | superproject | host test (all 65536 angles) + FOC/SINE build | done — F5 (AntiCogTriangle helper, 32-bit fold; estimator caveat documented, estimator fix out of scope) |
| T18 | UPSTREAM | superproject | review + FOC/SINE build | done — F22 (interpolation clamp) + F23 (SINGLE-mode direction-assumed comment) |
| T19 | UPSTREAM | superproject | review + FOC/SINE build | done — F24 (ocurlim ABS guard; deadtime DTG nonlinearity documented as comment — PARAM_ENTRY has no description field) |
| T9 | UPSTREAM | libopeninv (+super tests/bump) | host tests (4-combo matrix, exercises production code) + FOC/SINE build | done — F12 (all 3 defects fixed; pure packing logic split to canfilterpack.cpp so host tests drive real code — PR 5 may inline the split back if upstream prefers) |
| T10 | UPSTREAM | libopeninv (+super tests/bump) | host tests (4: normal, 1.5-period overrun, both wrap directions) + FOC/SINE build | done — F15 (missed deadline resyncs `TIM_CCR = counter + period` via `CheckOverrun`, a pure static split out for host testability — T9 precedent, raw TIM_CCR MMIO isn't host-drivable; overrun counter readable via `GetOverrunCount()`, no param) |
| T12 | UPSTREAM | both | host test (throttle IIR state isolation) + FOC/SINE build; items 2–4 review-verified | done — F16, 4 commits: throttle IIR state per caller (`FrequencyLimitCommandFw`, shared `RunFrequencyLimit` helper); `bmwAdcNextChan` wraps at `maxChan-1`; SDO frames DLC<8 rejected (libopeninv); cogging sentinels init 0 (first-crossing artifact; post-crossing resets keep ±INT32_MAX — a sample always lands between crossings). IIRFILTER rounding bias left unfixed — a metrology-polish item, not this pass's priority |
| T16 | UPSTREAM | both | review + host regression + FOC/SINE build (terminal glue, no host harness) | done — F18: terminal `defaults` now calls `Param::Change(PARAM_LAST)` (mirrors SDO path) and both `defaults` (super) and `load` (libopeninv) gate on `saveEnabled` via new `IsSaveEnabled()` accessor; messages match the `save` gate pattern |
| T13 | FORK (doc) | superproject | design doc only, no code | done — F3: `doc_sync_sampling_design.md`. Recommends TIM1_CC4 (JEXTSEL=1) injected trigger, Option A time-share (PWM ISR reads currents → hands injected group to resolver; JEOC ISR reads sin/cos → hands back) first, Option B (TIM1-locked excitation, single 2-deep sequence) as end-state. `syncadv`/`syncofs` re-tune required after A. Implementation gated on doc review |
| T14 | FORK (doc) | n/a | doc only; params cross-checked vs `param_prj.h` on fixes | done — parameter baseline for the EM57 build, written up as an internal doc (not part of this repo). Headline: `respolepairs=4 [VERIFY]` is the #1 first-spin trap (default 1 → 4× angle); `qlimfrq=0` + supervisory re-own of ALL throttle derates (F9) = the dyno mode; bus ladder 300/320/‹350 OBC›/‹360–375 dump›/385/430/‹450 HW›; `ocurlim`/`fmax`/ladder final values gate on HV power-stage selection |
| T20 | FORK (doc) | both | doc only, no code | done — public-facing README banners in both repos (this fork's purpose, the fixes/pin scheme, and the bench-validation disclosure) plus this rewrite of FORK_NOTES.md for outside readers |
| B2 | FORK (bench branch, **unmerged**) | superproject | host suite + FOC/SINE build; **NO HW PASS** | on `feat/B2-autotune` — port of upstream `35=AutoTune` (Pete's commissioning test modes via Jamie Jones, tip 1797847, upstream-untested): resolver check / phase check with inductance estimate / fwd+bidir test spins with sync-error spot values. Params renumbered to 167–171, values 2058–2060 (upstream's ids collided). Fixed F25 en route; added the missing entry path (`manualstart` + `testmode`≠0 → ManualRun via the normal start flow; physical start input always wins) and a `testres` terminal command for the phase-check verdict. Deliberately NOT merged to `fixes`: commissioning tooling, stock behavior unaffected only because the branch is separate |
| C1 | FORK (opt-in flag, **unmerged**) | superproject | flag-off byte-identity vs `fixes` (FOC + SINE, md5/cmp, independently re-verified) + host suite + flag-on FOC link + deliberate `#error` on SINE+flag; **NO HW PASS** | on `feat/C1-sync-sampling` — Option C.1 of `doc_sync_sampling_design.md` (finding **F3**), behind `#ifdef SYNC_CURRENT_SAMPLING` (`make … SYNC_SAMPLING=1`), default OFF: il1/il2 move to ADC2's injected group {IN5 dummy, IN5, IN8} triggered by TIM1_CC4 (OC4 PWM2, CCR4 = pwmmax−252, once-per-period OC4REF rising edge — bench-verified); AnaIn collapses to the library's single-ADC scan; resolver sin/cos both on ADC1 ranks 2/3 (pinswap becomes a rank pair, physical-pin defaults preserved); DUALMOD stays independent. HW_REV1 unsupported under the flag (il2-on-PA6 collides with resolver sin). Commissioning [VERIFY]: UEV top-vs-bottom RCR phase (sets CCR4 placement), then the doc §Q4 A/B noise-floor plan |
| T21 | FORK (opt-in flag, **merged to `fixes` 2026-07-19**) | superproject | flag-off byte-identity vs `fixes` (FOC + SINE, md5/cmp) + host suite (5 new cases) + flag-on FOC/SINE build; **NO HW PASS** | on `feat/T21-manualiq-autozero` — `manualiq` auto-zero on command-silence (D4 backstop), behind `#ifdef MANUALIQ_CMD_TIMEOUT` (`make CMD_TIMEOUT=1`), default OFF, FOC-only. Defence-in-depth BEHIND the GPIO dead-man: a bounded silence of the `manualiq`/`manualid` SDO command (stamped in T7's `Param::Change` fast-path) zeroes `manualiq` — a graceful command decay, NOT a fault trip (contactor/PWM-enable untouched, so a brief host stall recovers with no re-precharge). Timeout = param `iqtimeout` (id 172 [skips B2's 167-171], 10 ms ticks, 0=disabled, default 20=`PLACEHOLDER_HW`); detection semantics + timeout value are bench-gated → `PLACEHOLDERS.md` P1/P2. Decision logic split pure into `include/cmdtimeout.h` (`CmdTimeout`, RegenTaperHold/CheckOverrun precedent) so the host suite drives real code — `ApplyZero` zeroes the command past timeout with a contactor proxy untouched, + disabled/not-seen/refresh/wraparound guards; the FOC-only Param glue is firmware-compile-checked (host suite links the SINE param branch where `manualiq`/`iqtimeout` don't exist, same reason T7's manualiq path has no host test). `iqtimeout` added via an empty-expanding `CMD_TIMEOUT_PARAMS` macro (no `#ifdef` legal inside the `PARAM_LIST` backslash-macro). Amendment 500603a (ruled 2026-07-19): `manualid` zeroed too on expiry — a dead host must not leave flux current latched. Merged to `fixes` 973ce77 after full re-verification (flag-off byte-identical both variants, flag-on SINE inert, host suite green); flag default OFF keeps unvalidated behaviour off the default binary until bench validates P1/P2 |

| B4 | FORK (**merged to `fixes` 2026-07-20**) | superproject | FOC/SINE build (not byte-identical — behaviour change, expected); host suite green (unaffected — firmware `main`); scheduler T10 path covered by existing `test_stm32scheduler`; **NO HW PASS** | on `feat/B4-spread-can-tx` — port of upstream PR #58 (Modellfan, "spread mapped CAN TX slots"). Replaces the burst `canMap->SendAll()` in Ms10/Ms100Task with a `SendByIndex(canMapTxSlot)` round-robin (`canMapTxSlot = (canMapTxSlot+1) % MAX_MESSAGES`), one slot per tick, and adds a new `Ms1Task`. Mapping: 10 ms-period map → one slot per **1 ms** tick (new Ms1Task); 100 ms-period map → one slot per **10 ms** tick (Ms10Task). `CanMap::SendByIndex` was already merged into our libopeninv (no submodule bump — the c076e19→fa6e1dc bump in PR #58 is the same libopeninv merge we already carry; verified present). **Per-message refresh is unchanged**: MAX_MESSAGES=10 and both tick ratios are exactly 10:1, so a full cycle = 10 ms (10 ms mode) / 100 ms (100 ms mode), identical to `SendAll`; `SendByIndex` no-ops on empty slots (period stays MAX_MESSAGES×tick regardless of how many are mapped). **T10 (F15) interaction checked (jotham-todo flagged this the thinky bit): net-positive.** Moving a full TX burst OFF Ms10/Ms100 *reduces* their exec time → *improves* their overrun margin, not worsens it. New task is the 3rd scheduler channel (OC3); `MAX_TASKS=4`, one spare. Overrun accounting is per-channel and unchanged; the only nuance is the *shared* `overruns` counter now also counts any Ms1 skip if `Run()` is starved >1 ms (a busy PWM ISR) — diagnostic-fidelity only, never a control-path effect, and Ms1 does a single mailbox write so it won't overrun in normal operation. No new host test (matches PR #58 — timing/HW behaviour; `AddTask`/`CheckOverrun` already host-covered). **Merged to `fixes` 2026-07-20 (call delegated by Jotham): clearly beneficial (avoids flooding the 3 bxCAN TX mailboxes on a busy dyno bus, trims the heavy tasks) and low-risk; the deploy-freeze that motivated deferral lifted, and the whole `fixes` branch is pre-HW anyway — bench validation covers it with everything else.** |

Baseline pins → integration branch: superproject `1dfab85 → fixes`,
libopeninv `78e3f72 → fixes`.

(Append one row per task as branches merge to `fixes`.)
