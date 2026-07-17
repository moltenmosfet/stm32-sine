#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

//C1 (doc_sync_sampling_design.md, Option C.1): phase currents move off the
//regular AnaIn scan onto ADC2's injected group, PWM-synchronous via TIM1_CC4
//(see src/pwmgeneration.cpp TimerSetup, src/hwinit.cpp
//sync_current_sampling_setup). Only implemented for FOC -- SINE's current
//consumers (Charge/ProcessCurrents/LimitCurrent) stay on the AnaIn path.
#ifdef SYNC_CURRENT_SAMPLING
#if CONTROL != CTRL_FOC
#error "SYNC_CURRENT_SAMPLING requires CONTROL=FOC"
#endif
#endif

#ifdef SYNC_CURRENT_SAMPLING
#define ADC_COUNT 1
#else
#define ADC_COUNT 2
#endif

#if CONTROL == CTRL_SINE
#define NUM_SAMPLES 3
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC
#elif CONTROL == CTRL_FOC
#define NUM_SAMPLES 3
#define SAMPLE_TIME ADC_SMPR_SMP_1DOT5CYC
#endif // CONTROL

#ifdef SYNC_CURRENT_SAMPLING
//il1/il2 sampled via ADC2's injected group instead (see above); the
//regular scan collapses to ADC1 alone.
#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, GPIOC, 1) \
   ANA_IN_ENTRY(throttle2, GPIOC, 0) \
   ANA_IN_ENTRY(udc,       GPIOC, 3) \
   ANA_IN_ENTRY(tmpm,      GPIOC, 2) \
   ANA_IN_ENTRY(tmphs,     GPIOC, 4) \
   ANA_IN_ENTRY(uaux,      GPIOA, 3) \

//Alternative list. Must contain exactly the same names and number of
//entries as ANA_IN_LIST but may contain different IO pins

#define ANA_IN_LIST_BLUEPILL \
   ANA_IN_ENTRY(throttle1, GPIOA, 0) \
   ANA_IN_ENTRY(throttle2, GPIOA, 1) \
   ANA_IN_ENTRY(udc,       GPIOA, 2) \
   ANA_IN_ENTRY(tmpm,      GPIOA, 3) \
   ANA_IN_ENTRY(tmphs,     GPIOA, 4) \
   ANA_IN_ENTRY(uaux,      GPIOB, 1)
#else
#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, GPIOC, 1) \
   ANA_IN_ENTRY(throttle2, GPIOC, 0) \
   ANA_IN_ENTRY(udc,       GPIOC, 3) \
   ANA_IN_ENTRY(tmpm,      GPIOC, 2) \
   ANA_IN_ENTRY(tmphs,     GPIOC, 4) \
   ANA_IN_ENTRY(uaux,      GPIOA, 3) \
   ANA_IN_ENTRY(il1,       GPIOA, 5) \
   ANA_IN_ENTRY(il2,       GPIOB, 0) \

//Alternative list. Must contain exactly the same names and number of
//entries as ANA_IN_LIST but may contain different IO pins

#define ANA_IN_LIST_BLUEPILL \
   ANA_IN_ENTRY(throttle1, GPIOA, 0) \
   ANA_IN_ENTRY(throttle2, GPIOA, 1) \
   ANA_IN_ENTRY(udc,       GPIOA, 2) \
   ANA_IN_ENTRY(tmpm,      GPIOA, 3) \
   ANA_IN_ENTRY(tmphs,     GPIOA, 4) \
   ANA_IN_ENTRY(uaux,      GPIOB, 1) \
   ANA_IN_ENTRY(il1,       GPIOA, 5) \
   ANA_IN_ENTRY(il2,       GPIOB, 0)
#endif // SYNC_CURRENT_SAMPLING

#endif // ANAIN_PRJ_H_INCLUDED
