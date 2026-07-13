/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2026 Molten MOSFET
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Inspection surface for the libopencm3 stubs. Test code includes this to
 * check how the firmware programmed the CAN filter banks (T9) and to drive the
 * stubbed timer counter / observe compare-value writes (T10). */
#ifndef STUB_LIBOPENCM3_H
#define STUB_LIBOPENCM3_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- CAN filter-bank programming record --- */
enum stub_filter_kind
{
   STUB_FILTER_NONE = 0,
   STUB_FILTER_LIST16,   /* can_filter_id_list_16bit_init: args = id1..id4      */
   STUB_FILTER_MASK16,   /* can_filter_id_mask_16bit_init: args = id1,mask1,id2,mask2 */
   STUB_FILTER_LIST32    /* can_filter_id_list_32bit_init: args = id1,id2       */
};

#define STUB_MAX_FILTER_CALLS 64

struct stub_filter_call
{
   int      kind;      /* enum stub_filter_kind */
   uint32_t nr;        /* filter bank number */
   uint32_t fifo;      /* target FIFO */
   int      enable;    /* bank enabled */
   uint32_t args[4];   /* ids/masks, meaning per kind (see above) */
};

extern struct stub_filter_call stub_filter_calls[STUB_MAX_FILTER_CALLS];
extern int stub_filter_call_count;

/** Clear the recorded CAN filter programming. Call before a test that inspects it. */
void stub_reset_filters(void);

/* --- Timer stubs (settable counter, recorded compare writes) --- */
#define STUB_NUM_OC 7   /* enum tim_oc_id spans TIM_OC1(0)..TIM_OC4(6) */

extern uint32_t stub_timer_counter;          /* returned by timer_get_counter */
extern uint32_t stub_timer_oc_value[STUB_NUM_OC]; /* last timer_set_oc_value per oc_id */

/** Reset the timer stub state (counter to 0, all compare values to 0). */
void stub_reset_timer(void);

#ifdef __cplusplus
}
#endif

#endif /* STUB_LIBOPENCM3_H */
