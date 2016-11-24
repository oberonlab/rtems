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

#ifndef LIBBSP_ARM_MIL1986BE9X_SPI_H
#define LIBBSP_ARM_MIL1986BE9X_SPI_H

#include <rtems.h>
#include <sys/ioctl.h>

int spi_initialize(void);
int spi_finalize(void);

#define SPI_IOCTL_SET_TIMEOUT	_IOW('S', 1, uint32_t)

#endif /* !LIBBSP_ARM_MIL1986BE9X_SPI_H */
