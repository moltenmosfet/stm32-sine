# ADC dual-mode gate tests (Option C, doc_sync_sampling_design.md)

Standalone Blue Pill (STM32F103C8) rig that answers the two register-level
questions blocking Option C of `../../doc_sync_sampling_design.md`: do the two
ADCs' injected groups run truly independent triggers under `DUALMOD = RSM`
(regular simultaneous only), and does the dual-packed regular scan survive
one-sided injected pre-emption?

## Wiring

- ST-LINK on SWD (flash + semihosting output; a 3.3 V USART1/PA9 adapter at
  115200 8N1 shows the same output).
- PA5 -> 3.3 V and PB0 -> GND: these are il1/il2's ADC channels (ADC12_IN5/IN8),
  pinned to opposite rails so a wrong-channel or swapped-slot sample is
  unmistakable. PA6/PA7 stay floating (resolver stand-ins, values unused).
- BOOT0/BOOT1 = 0.

## Run

```
make
make flash-ocd    # or: make flash (st-flash)
# live output:
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c "init; arm semihosting enable; reset run"
```

One line prints per 2 s phase: A = both injected triggers on, B = TIM3_CC4
(ADC1/resolver stand-in) off, C = TIM1_CC4 (ADC2/currents) off, D = both off
(pairing baseline).

## Results (16 Jul 2026, F103C8, 72 MHz, ADCCLK 12 MHz)

- **Gate 1 PASS:** under RSM each injected group fires only on its own trigger
  (j1 = 2000/2 s @ 1 kHz TIM3_CC4; j2 = 35157/2 s @ 17.578 kHz TIM1_CC4; killing
  a trigger silences only its own group; the small phase-boundary residuals are
  the ~15 ms semihosting print latency). No JDR cross-contamination.
- **Gate 2 FAIL for the dual-packed scan:** baseline phase D showed 7.7 M packed
  words with zero pairing errors, but with either ADC's injected group
  pre-empting, the ADC2 half of the packed word carried the *neighboring
  channel's* value 45-54 % of the time in runs up to ~2 ms (`swapmax` = 1737
  poll-loop sweeps). Cross-channel substitution, not staleness -> Option C must
  move all housekeeping regulars to ADC1 alone ("C.1" in the design doc).
- The TIM1_CC4 injected trigger fires once per PWM period (OC4REF rising edge),
  not twice per compare match. The serviced update event landed at the counter
  top (DIR read in the update ISR) — re-check in-firmware, the rig skips
  TimerSetup's `TIM_EGR_UG` kick, which can shift the RCR phase alignment.
