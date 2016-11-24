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

#include <rtems.h>
#include <bsp/bootcard.h>
#include <bsp/1986be9x.h>

void bsp_reset(void)
{
  rtems_interrupt_level level;

  (void) level;
  rtems_interrupt_disable(level);

  /* Trigger watchdog reset */
  IWDG->KR = 0x5555;
  IWDG->PRL = 1;
  IWDG->KR = 0xcccc;

  while (true) {
    /* Do nothing */
  }
}
