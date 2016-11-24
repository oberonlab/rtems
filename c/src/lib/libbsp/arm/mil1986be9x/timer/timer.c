/*
 * Copyright (c) 2016 Andrey Mitrofanov <avm@oberonlab.ru>.
 *
 * JSC KB OBERON
 * St.Petersburg
 * Russia
 * http://www.oberonlab.ru
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <rtems/btimer.h>
#include <bsp/1986be9x.h>

static bool benchmark_timer_find_average_overhead = false;

static uint32_t get_val()
{
	return (((TIMER2->CNT)&0xFFFF) << 16)+((TIMER1->CNT)&0xFFFF);
}

void benchmark_timer_initialize(void)
{
	TIMER2->CNT=TIMER1->CNT=0;
}

uint32_t benchmark_timer_read(void)
{
	return get_val();
}

void benchmark_timer_disable_subtracting_average_overhead(
		bool find_average_overhead)
{
	benchmark_timer_find_average_overhead = find_average_overhead;
}

