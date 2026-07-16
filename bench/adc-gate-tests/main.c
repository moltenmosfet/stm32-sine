/* Option C gate tests for stm32-sine doc_sync_sampling_design.md (T13 addendum).
 *
 * Target: Blue Pill (STM32F103C8), ST-LINK flash, results on USART1 PA9 TX
 * @ 115200 8N1. Replicates the firmware's fabric: 72 MHz from 8 MHz HSE,
 * ADCCLK = PCLK2/6 = 12 MHz, TIM1 center-aligned period 2048 RCR 3 (UEV at
 * 8789 Hz, CC4 events at 2x17.578 kHz), dual-ADC mode.
 *
 * Gate 1 (RSM injected independence):
 *   DUALMOD = RSM (regular simultaneous only). ADC1 injected {ch6,ch7}
 *   triggered by TIM3_CC4 @ 1 kHz; ADC2 injected {ch5,ch8} triggered by
 *   TIM1_CC4. Phase machine alternately kills each trigger source:
 *     A: both on          -> expect j1 ~= 2000/2s, j2 ~= 35157/2s
 *     B: TIM3 CC4 off     -> expect j1 ~= 0,       j2 unchanged
 *     C: TIM1 CC4 off     -> expect j1 unchanged,  j2 ~= 0
 *     D: both off         -> regular-pairing baseline, no pre-emption
 *   PASS = each counter follows ONLY its own trigger.
 *
 * Gate 2 (regular-scan integrity under one-sided injected pre-emption):
 *   Dual regular simultaneous scan, ADC1 {ch16 temp, ch17 vref}, ADC2
 *   {ch5, ch8}, circular DMA of packed 32-bit ADC1_DR words. With PA5
 *   jumpered to 3V3 and PB0 to GND, corruption / slot-swap is any sample
 *   where slot0's ADC2 half is not ~4095 or slot1's is not ~0 (and ADC1
 *   halves must stay mid-scale). Violation counters printed per phase.
 *
 * Bonus [VERIFY]s: TIM1 counter value + count direction captured at every
 * ADC2 JEOC -> which slope the sample lands on and the CC4->JEOC latency.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#define PWM_PERIOD   2048
#define TRIG_LEAD    2006   /* CCR4: ~42 ticks (0.58 us) before the top */

/* ---- shared with ISRs ---- */
static volatile uint32_t ms;
static volatile uint32_t uev_cnt, j1_cnt, j2_cnt;
static volatile uint16_t j1_jdr1, j1_jdr2, j2_jdr1, j2_jdr2;
static volatile uint16_t j2cnt_min_up = 0xFFFF, j2cnt_max_up;
static volatile uint16_t j2cnt_min_dn = 0xFFFF, j2cnt_max_dn;
static volatile uint32_t uev_dir_up, uev_dir_dn; /* CR1.DIR at update irq */
static volatile uint32_t dma_buf[2]; /* packed: ADC2<<16 | ADC1, 2 slots */

void sys_tick_handler(void) { ms++; }

void tim1_up_isr(void)
{
   timer_clear_flag(TIM1, TIM_SR_UIF);
   uev_cnt++;
   if (TIM_CR1(TIM1) & TIM_CR1_DIR_DOWN) uev_dir_dn++; else uev_dir_up++;
}

void adc1_2_isr(void)
{
   if (ADC_SR(ADC1) & ADC_SR_JEOC)
   {
      ADC_SR(ADC1) &= ~ADC_SR_JEOC;
      j1_cnt++;
      j1_jdr1 = adc_read_injected(ADC1, 1);
      j1_jdr2 = adc_read_injected(ADC1, 2);
   }
   if (ADC_SR(ADC2) & ADC_SR_JEOC)
   {
      ADC_SR(ADC2) &= ~ADC_SR_JEOC;
      j2_cnt++;
      j2_jdr1 = adc_read_injected(ADC2, 1);
      j2_jdr2 = adc_read_injected(ADC2, 2);
      uint16_t cnt = timer_get_counter(TIM1);
      if (TIM_CR1(TIM1) & TIM_CR1_DIR_DOWN)
      {
         if (cnt < j2cnt_min_dn) j2cnt_min_dn = cnt;
         if (cnt > j2cnt_max_dn) j2cnt_max_dn = cnt;
      }
      else
      {
         if (cnt < j2cnt_min_up) j2cnt_min_up = cnt;
         if (cnt > j2cnt_max_up) j2cnt_max_up = cnt;
      }
   }
}

/* ---- output: USART1 always; semihosting line-flush when a debugger is
 * attached (openocd: `arm semihosting enable`). ---- */
#define DHCSR (*(volatile uint32_t *)0xE000EDF0)

static void sh_write0(const char *s)
{
   register uint32_t r0 __asm__("r0") = 0x04; /* SYS_WRITE0 */
   register const char *r1 __asm__("r1") = s;
   __asm__ volatile ("bkpt 0xAB" : : "r"(r0), "r"(r1) : "memory");
}

static char linebuf[192];
static unsigned linelen;

static void puts_(const char *s)
{
   for (; *s; s++)
   {
      if (*s == '\n') usart_send_blocking(USART1, '\r');
      usart_send_blocking(USART1, *s);
      if (linelen < sizeof(linebuf) - 2) linebuf[linelen++] = *s;
      if (*s == '\n')
      {
         linebuf[linelen] = 0;
         if (DHCSR & 1) sh_write0(linebuf); /* C_DEBUGEN set */
         linelen = 0;
      }
   }
}

static void putu(uint32_t v)
{
   char b[11];
   int i = 10;
   b[i] = 0;
   do { b[--i] = '0' + v % 10; v /= 10; } while (v && i);
   puts_(&b[i]);
}

static void kv(const char *k, uint32_t v) { puts_(k); putu(v); }

/* ---- setup ---- */
static void adc_setup_one(uint32_t adc)
{
   adc_power_off(adc);
   adc_enable_scan_mode(adc);
   adc_set_continuous_conversion_mode(adc);
   adc_set_right_aligned(adc);
   adc_set_sample_time_on_all_channels(adc, ADC_SMPR_SMP_71DOT5CYC);
   adc_power_on(adc);
   for (volatile int i = 0; i < 80000; i++); /* startup, mirrors firmware */
   adc_reset_calibration(adc);
   adc_calibrate(adc);
}

int main(void)
{
   rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
   rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_USART1);
   rcc_periph_clock_enable(RCC_ADC1);
   rcc_periph_clock_enable(RCC_ADC2);
   rcc_periph_clock_enable(RCC_TIM1);
   rcc_periph_clock_enable(RCC_TIM3);
   rcc_periph_clock_enable(RCC_DMA1);

   /* LED on PC13 as heartbeat */
   gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

   /* USART1 TX on PA9 */
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
   usart_set_baudrate(USART1, 115200);
   usart_set_databits(USART1, 8);
   usart_set_stopbits(USART1, USART_STOPBITS_1);
   usart_set_parity(USART1, USART_PARITY_NONE);
   usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
   usart_set_mode(USART1, USART_MODE_TX);
   usart_enable(USART1);

   /* analog pins: PA5(ch5, ->3V3), PA6/PA7(ch6/7, float), PB0(ch8, ->GND) */
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO5 | GPIO6 | GPIO7);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);

   /* ---- ADCs ---- */
   adc_setup_one(ADC1);
   adc_setup_one(ADC2);
   adc_enable_temperature_sensor();
   /* long sampling for temp/vref internal channels */
   adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_239DOT5CYC);
   adc_set_sample_time(ADC1, ADC_CHANNEL_VREF, ADC_SMPR_SMP_239DOT5CYC);
   /* NOTE: firmware uses 1.5 cyc on the current channels (low-Z sensor
    * outputs). Here the sources are jumper wires of unknown contact
    * resistance, so sample generously — we are testing the trigger/dual-mode
    * fabric, not analog settling. */
   adc_set_sample_time(ADC2, 5, ADC_SMPR_SMP_71DOT5CYC);
   adc_set_sample_time(ADC2, 8, ADC_SMPR_SMP_71DOT5CYC);
   adc_set_sample_time(ADC1, 6, ADC_SMPR_SMP_71DOT5CYC);
   adc_set_sample_time(ADC1, 7, ADC_SMPR_SMP_71DOT5CYC);

   /* regular groups: dual simultaneous scan, 2 slots each */
   uint8_t seq1[] = { ADC_CHANNEL_TEMP, ADC_CHANNEL_VREF };
   uint8_t seq2[] = { 5, 8 };
   adc_set_regular_sequence(ADC1, 2, seq1);
   adc_set_regular_sequence(ADC2, 2, seq2);
   adc_enable_dma(ADC1);
   adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
   adc_enable_external_trigger_regular(ADC2, ADC_CR2_EXTSEL_SWSTART);

   /* injected groups: the crux. Independent triggers under RSM. */
   uint8_t jseq1[] = { 6, 7 };
   uint8_t jseq2[] = { 5, 8 };
   adc_set_injected_sequence(ADC1, 2, jseq1);
   adc_set_injected_sequence(ADC2, 2, jseq2);
   adc_enable_external_trigger_injected(ADC1, ADC_CR2_JEXTSEL_TIM3_CC4);
   adc_enable_external_trigger_injected(ADC2, ADC_CR2_JEXTSEL_TIM1_CC4);
   adc_enable_eoc_interrupt_injected(ADC1);
   adc_enable_eoc_interrupt_injected(ADC2);
   nvic_enable_irq(NVIC_ADC1_2_IRQ);

   /* DMA: 2 packed words from ADC1_DR, circular */
   dma_set_peripheral_address(DMA1, 1, (uint32_t)&ADC_DR(ADC1));
   dma_set_memory_address(DMA1, 1, (uint32_t)dma_buf);
   dma_set_peripheral_size(DMA1, 1, DMA_CCR_PSIZE_32BIT);
   dma_set_memory_size(DMA1, 1, DMA_CCR_MSIZE_32BIT);
   dma_set_number_of_data(DMA1, 1, 2);
   dma_enable_memory_increment_mode(DMA1, 1);
   dma_enable_circular_mode(DMA1, 1);
   dma_enable_channel(DMA1, 1);

   /* THE mode under test */
   adc_set_dual_mode(ADC_CR1_DUALMOD_RSM);

   adc_start_conversion_regular(ADC1);

   /* ---- TIM1: firmware-shaped PWM timer ---- */
   timer_set_alignment(TIM1, TIM_CR1_CMS_CENTER_1);
   timer_set_period(TIM1, PWM_PERIOD);
   timer_set_repetition_counter(TIM1, 3);
   timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM2);
   timer_set_oc_value(TIM1, TIM_OC4, TRIG_LEAD);
   timer_enable_oc_output(TIM1, TIM_OC4);
   timer_enable_break_main_output(TIM1); /* no AF pins configured; nothing driven */
   timer_enable_irq(TIM1, TIM_DIER_UIE);
   nvic_enable_irq(NVIC_TIM1_UP_IRQ);
   timer_enable_counter(TIM1);

   /* ---- TIM3: 1 kHz CC4 trigger for the resolver stand-in ---- */
   timer_set_prescaler(TIM3, 71);   /* 1 MHz */
   timer_set_period(TIM3, 999);     /* 1 kHz */
   timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM2);
   timer_set_oc_value(TIM3, TIM_OC4, 500);
   timer_enable_oc_output(TIM3, TIM_OC4);
   timer_enable_counter(TIM3);

   systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
   systick_set_reload(72000 - 1);   /* 1 ms */
   systick_interrupt_enable();
   systick_counter_enable();

   puts_("\n== Option C gate tests (RSM injected independence) ==\n");
   puts_("expect/2s: uev~17578 j1~2000 j2~70312; J2=~4095,~0 if jumpers right\n");
   puts_("phases: A=both B=TIM3off C=TIM1off D=both-off(baseline)\n\n");

   uint32_t last_ms = 0, l_uev = 0, l_j1 = 0, l_j2 = 0;
   uint32_t viol_a1 = 0, ok2 = 0, swap2 = 0, bad2 = 0, sweeps = 0;
   uint32_t swaprun = 0, swapmax = 0;
   uint16_t a2s0_min = 0xFFFF, a2s0_max = 0, a2s1_min = 0xFFFF, a2s1_max = 0;
   char phase = 'A';

   while (1)
   {
      /* gate-2 corruption scan between prints */
      uint32_t w0 = dma_buf[0], w1 = dma_buf[1];
      uint16_t a1s0 = w0 & 0xFFFF, a2s0 = w0 >> 16;
      uint16_t a1s1 = w1 & 0xFFFF, a2s1 = w1 >> 16;
      sweeps++;
      /* ADC2 halves: slot0 = PA5@3V3 high, slot1 = PB0@GND low.
       * Classify: correct / slots-swapped (sticky sequence slip) / other. */
      if (a2s0 >= 3900 && a2s1 <= 200) { ok2++; swaprun = 0; }
      else if (a2s0 <= 200 && a2s1 >= 3900)
      {
         swap2++;
         swaprun++;
         if (swaprun > swapmax) swapmax = swaprun;
      }
      else { bad2++; swaprun = 0; }
      /* ADC1 halves: temp & vref both live mid-scale */
      if (a1s0 < 800 || a1s0 > 2600 || a1s1 < 800 || a1s1 > 2600) viol_a1++;
      /* per-slot envelopes for the ADC2 halves */
      if (a2s0 < a2s0_min) a2s0_min = a2s0;
      if (a2s0 > a2s0_max) a2s0_max = a2s0;
      if (a2s1 < a2s1_min) a2s1_min = a2s1;
      if (a2s1 > a2s1_max) a2s1_max = a2s1;

      if (ms - last_ms >= 2000)
      {
         last_ms = ms;
         uint32_t uev = uev_cnt, j1 = j1_cnt, j2 = j2_cnt;

         char pb[2] = { phase, 0 };
         puts_("["); puts_(pb); puts_("] ");
         kv("uev=", uev - l_uev);
         kv(" j1=", j1 - l_j1);
         kv(" j2=", j2 - l_j2);
         kv(" | J1=", j1_jdr1); kv(",", j1_jdr2);
         kv(" J2=", j2_jdr1); kv(",", j2_jdr2);
         kv(" | R0=", dma_buf[0] & 0xFFFF); kv("/", dma_buf[0] >> 16);
         kv(" R1=", dma_buf[1] & 0xFFFF); kv("/", dma_buf[1] >> 16);
         kv(" a1viol=", viol_a1);
         kv(" ok/swap/bad=", ok2); kv("/", swap2); kv("/", bad2);
         kv(" sweeps=", sweeps);
         kv(" a2s0=", a2s0_min); kv("-", a2s0_max);
         kv(" a2s1=", a2s1_min); kv("-", a2s1_max);
         kv(" swapmax=", swapmax);
         kv(" | j2@up=", j2cnt_min_up); kv("-", j2cnt_max_up);
         kv(" dn=", j2cnt_min_dn); kv("-", j2cnt_max_dn);
         kv(" | uevdir up/dn=", uev_dir_up); kv("/", uev_dir_dn);
         puts_("\n");

         l_uev = uev; l_j1 = j1; l_j2 = j2;
         viol_a1 = ok2 = swap2 = bad2 = 0; sweeps = 0;
         swaprun = swapmax = 0;
         a2s0_min = a2s1_min = 0xFFFF; a2s0_max = a2s1_max = 0;
         j2cnt_min_up = j2cnt_min_dn = 0xFFFF;
         j2cnt_max_up = j2cnt_max_dn = 0;
         gpio_toggle(GPIOC, GPIO13);

         /* advance phase machine */
         if (phase == 'A')
         {
            phase = 'B';
            timer_disable_oc_output(TIM3, TIM_OC4);
         }
         else if (phase == 'B')
         {
            phase = 'C';
            timer_enable_oc_output(TIM3, TIM_OC4);
            timer_disable_oc_output(TIM1, TIM_OC4);
         }
         else if (phase == 'C')
         {
            /* D: no injected activity at all — regular-pairing baseline */
            phase = 'D';
            timer_disable_oc_output(TIM3, TIM_OC4);
         }
         else
         {
            phase = 'A';
            timer_enable_oc_output(TIM1, TIM_OC4);
            timer_enable_oc_output(TIM3, TIM_OC4);
         }
      }
   }
}
