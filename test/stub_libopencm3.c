/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2024 David J. Fiddes <D.J@fiddes.net>
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
#include "stdint.h"
#include <string.h>
#include "stub_libopencm3.h"

void flash_unlock(void)
{
}

void flash_lock(void)
{
}

void flash_set_ws(uint32_t ws)
{
}

void flash_program_word(uint32_t address, uint32_t data)
{
}

void flash_erase_page(uint32_t page_address)
{
}

uint16_t desig_get_flash_size(void)
{
    return 8;
}

uint32_t crc_calculate(uint32_t data)
{
    return 0xaa55;
}

/* ------------------------------------------------------------------ */
/* CAN filter-bank programming record (used by the T9 filter tests)    */
/* ------------------------------------------------------------------ */

struct stub_filter_call stub_filter_calls[STUB_MAX_FILTER_CALLS];
int stub_filter_call_count = 0;

void stub_reset_filters(void)
{
    memset(stub_filter_calls, 0, sizeof(stub_filter_calls));
    stub_filter_call_count = 0;
}

static struct stub_filter_call* stub_next_filter_call(void)
{
    if (stub_filter_call_count >= STUB_MAX_FILTER_CALLS)
        return &stub_filter_calls[STUB_MAX_FILTER_CALLS - 1]; /* saturate, never overflow */
    return &stub_filter_calls[stub_filter_call_count++];
}

/* Signatures mirror libopencm3 <libopencm3/stm32/can.h>; bool is int here. */
void can_filter_id_list_16bit_init(uint32_t nr, uint16_t id1, uint16_t id2,
                                   uint16_t id3, uint16_t id4, uint32_t fifo,
                                   int enable)
{
    struct stub_filter_call* c = stub_next_filter_call();
    c->kind = STUB_FILTER_LIST16;
    c->nr = nr; c->fifo = fifo; c->enable = enable;
    c->args[0] = id1; c->args[1] = id2; c->args[2] = id3; c->args[3] = id4;
}

void can_filter_id_mask_16bit_init(uint32_t nr, uint16_t id1, uint16_t mask1,
                                   uint16_t id2, uint16_t mask2, uint32_t fifo,
                                   int enable)
{
    struct stub_filter_call* c = stub_next_filter_call();
    c->kind = STUB_FILTER_MASK16;
    c->nr = nr; c->fifo = fifo; c->enable = enable;
    c->args[0] = id1; c->args[1] = mask1; c->args[2] = id2; c->args[3] = mask2;
}

void can_filter_id_list_32bit_init(uint32_t nr, uint32_t id1, uint32_t id2,
                                   uint32_t fifo, int enable)
{
    struct stub_filter_call* c = stub_next_filter_call();
    c->kind = STUB_FILTER_LIST32;
    c->nr = nr; c->fifo = fifo; c->enable = enable;
    c->args[0] = id1; c->args[1] = id2; c->args[2] = 0; c->args[3] = 0;
}

/* ------------------------------------------------------------------ */
/* Timer stubs (settable counter, recorded compare writes) for T10     */
/* ------------------------------------------------------------------ */

uint32_t stub_timer_counter = 0;
uint32_t stub_timer_oc_value[STUB_NUM_OC];

/* Stm32Scheduler's own APB1 clock arithmetic (constructor) isn't under
 * test; libopencm3 normally defines this in code we don't compile here. */
uint32_t rcc_apb1_frequency = 36000000;

void stub_reset_timer(void)
{
    stub_timer_counter = 0;
    memset(stub_timer_oc_value, 0, sizeof(stub_timer_oc_value));
}

uint32_t timer_get_counter(uint32_t timer_peripheral)
{
    (void)timer_peripheral;
    return stub_timer_counter;
}

void timer_set_counter(uint32_t timer_peripheral, uint32_t count)
{
    (void)timer_peripheral;
    stub_timer_counter = count;
}

/* oc_id is enum tim_oc_id (int) in libopencm3; guard the index defensively. */
void timer_set_oc_value(uint32_t timer_peripheral, int oc_id, uint32_t value)
{
    (void)timer_peripheral;
    if (oc_id >= 0 && oc_id < STUB_NUM_OC)
        stub_timer_oc_value[oc_id] = value;
}

/* Remaining timer_* calls made by Stm32Scheduler's constructor/AddTask/Run
 * are register setup/IRQ plumbing that Stm32Scheduler.cpp needs to link on
 * the host; the tests below drive Stm32Scheduler::CheckOverrun (a pure
 * function on plain values, no register access), so these are no-ops. */
void timer_enable_preload(uint32_t timer_peripheral) { (void)timer_peripheral; }
void timer_direction_up(uint32_t timer_peripheral) { (void)timer_peripheral; }
void timer_set_prescaler(uint32_t timer_peripheral, uint32_t value)
{
    (void)timer_peripheral; (void)value;
}
void timer_set_period(uint32_t timer_peripheral, uint32_t period)
{
    (void)timer_peripheral; (void)period;
}
void timer_set_oc_mode(uint32_t timer_peripheral, int oc_id, int oc_mode)
{
    (void)timer_peripheral; (void)oc_id; (void)oc_mode;
}
void timer_enable_irq(uint32_t timer_peripheral, uint32_t irq)
{
    (void)timer_peripheral; (void)irq;
}
void timer_enable_counter(uint32_t timer_peripheral) { (void)timer_peripheral; }
void timer_disable_counter(uint32_t timer_peripheral) { (void)timer_peripheral; }
int timer_get_flag(uint32_t timer_peripheral, uint32_t flag)
{
    (void)timer_peripheral; (void)flag;
    return 0;
}
void timer_clear_flag(uint32_t timer_peripheral, uint32_t flag)
{
    (void)timer_peripheral; (void)flag;
}
