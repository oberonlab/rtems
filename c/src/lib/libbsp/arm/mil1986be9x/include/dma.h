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

#ifndef LIBBSP_ARM_MIL1986BE9X_DMA_H
#define LIBBSP_ARM_MIL1986BE9X_DMA_H

#define MIL1986BE9X_DMA_NUM		32
#define MIL1986BE9X_DMA_UART1_TX	0
#define MIL1986BE9X_DMA_UART1_RX	1
#define MIL1986BE9X_DMA_UART2_TX	2
#define MIL1986BE9X_DMA_UART2_RX	3
#define MIL1986BE9X_DMA_SSP1_TX		4
#define MIL1986BE9X_DMA_SSP1_RX		5
#define MIL1986BE9X_DMA_SSP2_TX		6
#define MIL1986BE9X_DMA_SSP2_RX		7
#define MIL1986BE9X_DMA_TIMER1		10
#define MIL1986BE9X_DMA_TIMER2		11
#define MIL1986BE9X_DMA_TIMER3		12

rtems_status_code mil1986be9x_dma_initialize(void);
rtems_status_code mil1986be9x_dma_channel_obtain(unsigned channel);
rtems_status_code mil1986be9x_dma_channel_release(unsigned channel);
rtems_status_code mil1986be9x_dma_copy(unsigned channel, void *dst,
				       const void *src, size_t n,
				       size_t dst_width, size_t src_width);

rtems_status_code mil1986be9x_dma_copy_async(unsigned channel, void *dst,
				       const void *src, size_t n,
				       size_t dst_width, size_t src_width,
				       void (*complete)(void *), void *arg);
#endif /* LIBBSP_ARM_MIL1986BE9X_DMA_H */
