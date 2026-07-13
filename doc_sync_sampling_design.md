# PWM-synchronous phase-current sampling — design investigation (F3 / T13)

Status: **design-only, decision-ready.** No code changed. Target: STM32F103 (RM0008),
this fork's dyno-main. Audience: fork maintainers. All line refs verified on `dyno-main`
(superproject `a7af5a4`, libopeninv `1114748`).

## Problem (F3 recap, grounded)

Phase currents il1/il2 are sampled by the **regular** ADC group in continuous
free-running scan with circular DMA (`libopeninv/src/anain.cpp:57-89`, `Get()` returns a
median-of-3, `libopeninv/src/anain.cpp:113-118`). Sampling is unsynchronised to the PWM,
so each read lands at a random phase of the current-ripple triangle. `ProcessCurrents`
consumes it via `GetCurrent(AnaIn::il1,…)` (`src/pwmgeneration-foc.cpp:241-242`,
`src/pwmgeneration.cpp:357-362`). At the EM57's sub-mH inductance, 8.8 kHz, 350–400 V bus,
ripple is tens of A p-p — comparable to the whole signal at 10 N·m — and aliases into
id/iq. Goal: sample il1/il2 at the PWM ripple **midpoint** (counter top/bottom of the
centre-aligned timer) so the sample equals the true average phase current.

## Hardware facts that constrain every option (verified)

- **Both current sensors are ADC1+ADC2 only.** il1 = PA5 = ADC12_IN5, il2 = PB0 =
  ADC12_IN8 (`include/anain_prj.h:23-24`, `libopeninv/src/anain.cpp:161-196`). Neither pin
  is an ADC3 channel (ADC3 reaches only PA0-3/PC0-3), so the currents **cannot** be moved
  off the ADC1/ADC2 pair. il1 lands on ADC1, il2 on ADC2 (`AnaIn::Configure` dual-mode
  index split, `libopeninv/src/anain.cpp:97-99`; il1 is `__COUNTER__` index 6 → even →
  ADC1, il2 index 7 → odd → ADC2).
- **The resolver already owns the injected group on the same pair.** `InitResolverMode`
  puts sin=PA6=IN6 on ADC1 and cos=PA7=IN7 on ADC2 as an injected sequence
  (`src/inc_encoder.cpp:460-505`), trigger `ADC_CR2_JEXTSEL_TIM3_CC4`
  (`src/inc_encoder.cpp:499`). sin/cos are also strictly ADC1/ADC2 pins (not ADC3).
- **Dual mode is already the combined mode.** `AnaIn::Start` sets
  `adc_set_dual_mode(ADC_CR1_DUALMOD_CRSISM)` (`libopeninv/src/anain.cpp:86`;
  `ADC_CR1_DUALMOD_CRSISM = 0x1`, RM0008 §11.9 ADC_CR1 DUALMOD = "combined regular
  simultaneous + injected simultaneous"). So injected-simultaneous is **already active**:
  ADC1 is master, ADC2 slave. Consequence (RM0008 §11.10.x, injected simultaneous mode):
  the injected conversion is started by **ADC1's** JEXTSEL trigger; ADC2 converts in
  lockstep and its own JEXTSEL is a don't-care (set to JSWSTART today,
  `src/inc_encoder.cpp:469`). il1/sin on the master, il2/cos on the slave — the layout is
  already correct for either consumer.
- **TIM1 (the PWM timer) OC4/CC4 is free.** Phases use OC1/OC2/OC3 (+ complementary N)
  only (`src/pwmgeneration.cpp:386-401,416-427`); the setup loop runs `TIM_OC1..TIM_OC3N`
  and never touches OC4. Center-aligned mode 1 (`timer_set_alignment(PWM_TIMER,
  TIM_CR1_CMS_CENTER_1)`, `src/pwmgeneration.cpp:413`), period `1<<pwmdigits`
  (default 11 → 2048, `src/pwmgeneration.cpp:449`, `include/hwdefs.h:11`).
- **Control ISR = 8789 Hz constant, via the repetition counter**
  (`src/pwmgeneration.cpp:371-459`, `FRQ_DIVIDER 8192`). At the default 11-bit/17.6 kHz
  PWM the RCR is 3 → the update ISR fires once per **two** PWM periods; at 8.8 kHz PWM
  RCR=1 → once per period (`repCounters[]`, `src/pwmgeneration.cpp:380`, and the comment
  block 373-379).
- **ADCCLK = PCLK2/6 = 12 MHz** (`src/hwinit.cpp:50`, `RCC_CFGR_ADCPRE_PCLK2_DIV6`).
  FOC sample time 1.5 cyc (`include/anain_prj.h:13`) → one injected conversion = 14 ADC
  cyc ≈ **1.17 µs**; ADC1+ADC2 convert in parallel so the il1/il2 *pair* is also ~1.17 µs.
  Trivial against the 113.8 µs ISR period.
- **JEXTSEL table for the ADC1/ADC2 injected group** (RM0008 §11.12.3 ADC_CR2 JEXTSEL;
  libopencm3 `stm32/f1/adc.h:209-223`): TIM1_TRGO=0, **TIM1_CC4=1**, TIM2_TRGO=2,
  TIM2_CC1=3, **TIM3_CC4=4**, TIM4_TRGO=5, EXTI15/TIM8_TRGO=6, JSWSTART=7. TIM8_CC4=4 is
  the **ADC3** injected set (`adc.h:234-248`) — not reachable by ADC1/ADC2, so "TIM8
  options" are irrelevant here.

## The crux

One injected group per ADC. One JEXTSEL and one JSQR per group. Two consumers that need
the group at **two different instants of the same PWM period**:

- currents: at the counter update-event (ripple midpoint), synchronous to TIM1;
- resolver: 40 µs after the excitation edge (`resolverSampleDelay = 40`,
  `src/inc_encoder.cpp:65`, TIM3 @ 1 MHz one-shot, armed at ISR entry
  `src/inc_encoder.cpp:550-559`), i.e. mid-period, **asynchronous to** the TIM1 counter
  phase because the edge is toggled in software at ISR entry
  (`src/inc_encoder.cpp:545-561`).

A single trigger with a 2-deep injected sequence cannot serve both, because the two
instants differ by ~40 µs and the two sequence slots convert only ~1.17 µs apart. There is
no second ADC to offload to. Therefore any solution either (A) **time-shares** the one
injected group across the period, or (B) **phase-locks** the resolver excitation to the
PWM counter so both instants collapse to one trigger.

---

## Q1 — Trigger routing for the current sample

Recommended trigger: **TIM1_CC4 (JEXTSEL = 1)** on ADC1 (master); ADC2 follows in
injected-simultaneous dual mode (no JEXTSEL programming needed on ADC2).

Rationale vs the alternatives:

- **TIM1_CC4 (OC4)** — OC4 is unused, so we get a *programmable* trigger phase for free.
  Set CCR4 so the injected conversion **completes just before** the update event the ISR
  services (≈ CCR4 = period − ⌈1.4 µs·72 MHz⌉ ticks on the relevant count direction), so
  JDR holds a midpoint-synchronous sample the instant the ISR reads it. Caveat: in
  centre-aligned mode CC4 matches **twice** per period (up and down). We want the match
  adjacent to the serviced update event; the other match's conversion is harmless (it just
  refreshes JDR earlier and is overwritten), but see the RCR interaction below.
- **TIM1_TRGO on update (MMS=update)** — fires *at* the update event, i.e. ~1.17 µs too
  **late**: the conversion would finish after the ISR has already read JDR and started the
  FOC math. Rejected as the current trigger. (TRGO stays free for other uses.)
- **TIM8** — ADC3-only injected trigger; not reachable from ADC1/ADC2. N/A.

RCR interaction to pin on the bench: at the default 11-bit PWM the ISR runs every *other*
PWM period, but CC4 fires *every* period. JDR is simply overwritten each period and the ISR
reads the freshest — correct average either way, but confirm on scope that the sample the
ISR consumes is the one from the serviced half. If exact 1-per-ISR triggering is wanted,
raise `pwmfrq` to run 8.8 kHz PWM (RCR=1) or gate CC4 in the handoff (Option A already
reprograms per ISR, so this is free there). **[VERIFY on scope]** the update event lands on
the counter underflow (bottom, all high-side on) vs overflow (top) for the shipped
CMS_CENTER_1 + RCR config — it sets the CCR4 value and the "which switching edge is
nearest" analysis, not the architecture.

Dual-mode note (RM0008 §11.10, injected simultaneous): program **only ADC1** JEXTSEL =
TIM1_CC4 and enable its injected external trigger; ADC2 stays JSWSTART and is driven by the
master. il1 must be the master (ADC1) channel and il2 the slave (ADC2) channel — already
true.

---

## Q2 — Conflict resolution

### Option A (recommended for the first cut) — time-share the injected group across two ISRs

Split the two consumers onto two interrupts so exactly one injected conversion is in flight
at a time:

1. **PWM update ISR (8.8 kHz), the current loop.** Injected group is currently pointed at
   il1/il2 with JEXTSEL=TIM1_CC4; CC4 fired ~1.4 µs before this update event, so JDR1(ADC1)
   /JDR1(ADC2) already hold midpoint-synchronous il1/il2. Read them, run FOC. On the way
   out, **hand the group to the resolver**: rewrite JSQR to {sin} on ADC1 / {cos} on ADC2
   (`adc_set_injected_sequence`), set ADC1 JEXTSEL=TIM3_CC4, toggle the excitation GPIO edge
   and arm the TIM3 one-shot — i.e. the existing `GetAngleResolver` arming body, relocated
   to ISR *exit*.
2. **ADC JEOC ISR (fires ~40 µs later when the resolver conversion completes).** Read
   sin/cos (`adc_read_injected`), run `DecodeAngle`/`UpdateTurns` to update `Encoder::angle`,
   then **hand the group back to the currents**: rewrite JSQR to {il1}/{il2}, set ADC1
   JEXTSEL=TIM1_CC4, load CCR4 for the pre-update trigger of the next period.

Why a helper (JEOC) interrupt is **intrinsic**, not incidental: the resolver conversion is
triggered at +40 µs by TIM3 and the current conversion must be re-armed before the CC4
trigger at ≈+112 µs, but the only control ISR sits at the period boundary. Something must
run in the (+41 µs, +112 µs) window to read the resolver result and repoint the group. JEOC
is the natural, deterministic hook (`adc_enable_eoc_interrupt_injected`, `adc1_2_isr`). No
polling, no blocking wait.

Register cost per period: 2 × (JSQR write ADC1 + JSQR write ADC2 + JEXTSEL write ADC1) +
CCR4 write + excitation GPIO toggle + TIM3 arm — a few dozen bus cycles, negligible at
8.8 kHz; one added ISR entry/exit (~1–2 µs). **JEXTSEL/JSQR are writable with ADON=1 as long
as no injected conversion is in progress** (RM0008 §11.12.3; the handoffs happen while the
group is idle, so this holds — enforce with an ownership state variable, below).

What Option A leaves **bit-identical**: the resolver analog excitation waveform, the 40 µs
sample delay, and the sin/cos demod math (`DecodeAngle`, `src/inc_encoder.cpp:579-604`).
What it **changes**: the resolver angle is now read ~mid-period instead of one full ISR
later. That shortens the angle latency the double-`syncadv` compensation is tuned for
(intentional double-apply, `src/pwmgeneration-foc.cpp:65` + the second apply; FORK_NOTES /
review F8). **`syncofs`/`syncadv` must be re-derived on the bench after this change** — a
parameter re-tune, not a redesign, but call it out as a required commissioning step.

Ownership guard: keep a `static volatile enum {OWN_CURRENT, OWN_RESOLVER} injOwner;` set at
each handoff and asserted at each read (read il1/il2 only if `injOwner==OWN_CURRENT` at ISR
entry, else flag a fault and skip). This turns any missed/mis-ordered trigger into a
detectable event instead of silent cross-channel garbage.

### Option B (target end-state) — phase-lock excitation, single combined trigger

Drive the resolver excitation edge from a TIM1-synchronous timer output instead of the
software GPIO toggle, placing the edge 40 µs before the serviced update event. Then the
sample instant (edge + 40 µs) coincides with the counter midpoint, and a **single**
TIM1_CC4-triggered injected conversion with a **2-deep sequence per ADC** — ADC1 {il1, sin},
ADC2 {il2, cos} — samples both at the correct instant (il1/il2 at the trigger, sin/cos
~1.17 µs later, still within the flat midpoint window). No JEOC ISR, no per-cycle JSQR/
JEXTSEL reprogramming, one trigger.

Cost: reworks excitation generation (GPIO→timer edge) and requires re-validating carrier
amplitude/phase and the `resolverMax-resolverMin > MIN_RES_AMP` gate
(`src/inc_encoder.cpp:585`). The 4.4 kHz carrier (one edge per 113.8 µs ISR) is preserved.
Higher payoff (lowest steady-state overhead) but touches the load-bearing angle source more
invasively.

### Recommendation

**Implement Option A first.** It isolates the current-SNR change, keeps the resolver
excitation/demod untouched, and its only resolver-side perturbation (angle-read latency →
`syncadv` re-tune) is a bench parameter step. Once A demonstrates the id/iq noise-floor win
on the dyno, **graduate to Option B** to shed the JEOC ISR and per-cycle reprogramming.
Rejected outright: moving either consumer to ADC3 (pins don't map) or to the regular group
(free-running/DMA-owned by AnaIn, can't be phase-triggered without evicting the other 8
channels).

---

## Q3 — Changes in ProcessCurrents, offset calibration, fallback

**ProcessCurrents** (`src/pwmgeneration-foc.cpp:234-257`): replace the two
`GetCurrent(AnaIn::ilX,…)` reads with reads of the injected data registers —
`adc_read_injected(ADC1, rank)` for il1, `adc_read_injected(ADC2, rank)` for il2 — keeping
the identical `offset`/`gain` math from `GetCurrent` (`src/pwmgeneration.cpp:357-362`) and
the existing `pinswap`/`ParkClarke` and `Param::SetFixed` tail. The median-of-3
(`libopeninv/src/anain.cpp:117-118`) is dropped by design: a midpoint-synchronous sample
does not need outlier rejection; add a light 1-pole IIR only if bench data shows residual
sample jitter. Introduce a small accessor (e.g. `PwmGeneration::GetPhaseCurrentRaw`) rather
than reaching into ADC registers from two files.

**Offset calibration** (`RunOffsetCalibration`, `src/pwmgeneration-foc.cpp:276-293`, uses
`AnaIn::il1.Get()`): it runs at standstill with outputs inactive, where there is no PWM
ripple, so free-running vs injected read the same DC zero — but it must read the **same
path** it will later use, i.e. injected conversions, so gain/offset and any per-group
peculiarity match. Two clean choices: (a) keep the software offset subtract (average injected
zero reads into `ilofs[]` exactly as today, just via `adc_read_injected`); or (b) push the
offset into the injected hardware offset registers **JOFR1/JOFR2**
(`adc_set_injected_offset`) so JDR returns signed current directly — the resolver already
uses JOFR for its own zeroing (`src/inc_encoder.cpp:488-489,497-498`). Recommend (a) for
parity with the fallback path and to keep the offset visible in `ilofs[]`. Note the
calibration must acquire the injected samples through the **same time-share machinery** (or
run before the resolver handoff is enabled), else it will read whatever channel currently
owns the group.

**Fallback (build-time):** gate the whole feature behind e.g. `#ifdef SYNC_CURRENT_SAMPLING`.
When undefined, `ProcessCurrents`, `RunOffsetCalibration`, the ISR handoff, and the resolver
init stay exactly as on master (AnaIn regular path). This (i) preserves a known-good
regression baseline, (ii) makes the A/B bench comparison a recompile, and (iii) de-risks the
upstream story — the sync path is fork policy, not forced on all users.

---

## Q4 — Risks and bench validation

**Bench validation plan (dyno):**
1. Build A (master, AnaIn) vs build B (`SYNC_CURRENT_SAMPLING`). Hold a steady low setpoint
   (`manualiq` ≈ 10–30 A) at a fixed shaft speed above the F1 clamp band. Log id/iq
   (`Param::id/iq`) at the 8.8 kHz ISR (or stream via CAN at max rate) and compute the FFT.
   **Expected:** broadband id/iq noise floor drops; specifically the PWM-ripple alias tones
   collapse. Metric: id/iq RMS ripple at fixed operating point, and the alias tone
   amplitudes.
2. Sweep setpoint 0→300 A; confirm the noise improvement is largest at the low end (where
   ripple ≈ signal) and that high-current accuracy is unregressed.
3. Resolver integrity regression: verify decoded angle vs an independent reference,
   `resolverMax-resolverMin` still clears `MIN_RES_AMP` (no `ERR_LORESAMP`,
   `src/inc_encoder.cpp:598-601`), and re-derive `syncadv`/`syncofs` (Option A latency
   change). Confirm no `ERR_HIRESOFS` at init (`src/inc_encoder.cpp:501-504`).
4. Offset-cal convergence: confirm `ilofs[]` lands at the same values as the AnaIn build.
5. Instrument the ownership guard and a JEOC-overrun counter across a long run; assert zero
   cross-channel faults.

**Failure modes to watch:**
- **Missed / mis-ordered trigger (Option A).** If the JEOC handoff is late, the CC4 current
  trigger fires while the group still holds resolver channels → il1/il2 read sin/cos (wildly
  wrong current → current-loop kick), or the resolver samples current channels → angle
  garbage. Mitigation: the `injOwner` guard + fault flag; keep the JEOC ISR short and at a
  priority above lower-rate tasks.
- **Resolver excitation-phase interaction.** Option A: JEOC-handoff jitter shifts *when* the
  group returns to the currents, not the resolver sample instant, so demod is unaffected as
  long as the guard holds. Option B: excitation edge placement error directly shifts the
  sample off the 40 µs peak → amplitude/phase error in `Atan2`. Validate the carrier on
  scope before trusting angle.
- **Sample-at-switching-edge noise.** If CCR4 places the sample too near a leg's switching
  transition, the injected conversion catches switching ringing instead of the flat
  midpoint. At high modulation the three legs switch at spread-out times and the clean
  window shrinks — a known midpoint-sampling limit. The dyno rarely runs high modulation at
  low speed (low back-EMF), so acceptable; still, tune CCR4 to the counter extreme (all-high
  or all-low instant) and confirm on scope. **[VERIFY]** which extreme the serviced update
  event sits on.
- **RCR/ISR-rate aliasing (default 11-bit PWM).** CC4 fires every PWM period but the ISR
  reads every other; confirm the consumed sample is from the serviced half (see Q1).
- **`syncadv` mistune after Option A** presenting as torque ripple / reduced modulation
  headroom — expected, resolved by the commissioning re-tune, not a bug.

---

## Open questions / [VERIFY]

- **[VERIFY on scope]** Update event = counter overflow (top) or underflow (bottom) under
  the shipped `TIM_CR1_CMS_CENTER_1` + RCR config. Sets the CCR4 value and the nearest-edge
  analysis; does not change the chosen architecture.
- **[VERIFY]** Exact CC4→conversion-complete lead time to load CCR4 (1.17 µs conversion +
  injected trigger latency); measure, don't compute blind.
- **Open:** whether the resolver angle-latency change under Option A is worth avoiding by
  reading sin/cos in the JEOC ISR but *applying* the angle with the same one-ISR delay as
  today (keeps `syncadv` untouched at the cost of throwing away the freshness gain). Decide
  at implementation from whether re-tuning `syncadv` is cheaper than the extra staleness
  bookkeeping.
- **Open:** Option B carrier integrity — can a TIM1-synchronous edge preserve the 4.4 kHz
  excitation cleanly enough that `resolverMax-resolverMin` margin is unchanged? Bench
  question, gates the A→B graduation.
- **Confirmed not a blocker:** ADC throughput (1.17 µs/conversion vs 113.8 µs period) and
  the dual-mode wiring (CRSISM already active, il1/sin on master, il2/cos on slave).
