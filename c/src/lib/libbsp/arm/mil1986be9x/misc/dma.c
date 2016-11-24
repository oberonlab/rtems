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

#define RTEMS_STATUS_CHECKS_USE_PRINTK

#include <rtems.h>
#include <rtems/status-checks.h>

#include <bsp/1986be9x.h>
#include <bsp/dma.h>
#include <bsp/irq.h>

#define DMA_CONTROL_CYCLE_CTRL(n)	((n) & 7)
#define DMA_CONTROL_NEXT_USEBURST(n)	(((n) & 1) << 3)
#define DMA_CONTROL_N_MINUS_1(n)	(((n) & 0x3ff) << 4)
#define DMA_CONTROL_R_POWER(n)		(((n) & 0xf) << 14)
#define DMA_CONTROL_SRC_PROT_CTRL(n)	(((n) & 7) << 18)
#define DMA_CONTROL_DST_PROT_CTRL(n)	(((n) & 7) << 21)
#define DMA_CONTROL_SRC_SIZE(n)		(((n) & 3) << 24)
#define DMA_CONTROL_SRC_INC(n)		(((n) & 3) << 26)
#define DMA_CONTROL_DST_SIZE(n)		(((n) & 3) << 28)
#define DMA_CONTROL_DST_INC(n)		(((n) & 3) << 30)

/**
 * @brief Table that indicates if a channel is currently occupied.
 */
static bool dma_channel_occupation[MIL1986BE9X_DMA_NUM];
static bool dma_initialized = false;

struct dma_control_data {
	uint32_t src_end;
	uint32_t dst_end;
	union {
		uint32_t value;
		struct control_bits {
			unsigned int cycle_ctrl    : 3;
			unsigned int next_useburst : 1;
			unsigned int n_minus_1     : 10;
			unsigned int R_power       : 4;
			unsigned int src_prot_ctrl : 3;
			unsigned int dst_prot_ctrl : 3;
			unsigned int src_size      : 2;
			unsigned int src_inc       : 2;
			unsigned int dst_size      : 2;
			unsigned int dst_inc       : 2;
		} bits;
	} control;
	uint32_t unused;
};

// FIXME: reassign buffer from static address 0x20008000 to dynamic address
//static volatile struct dma_control_data dma_control[2 * MIL1986BE9X_DMA_NUM] __attribute__((at(0x20005000)));
static volatile struct dma_control_data *dma_control = (volatile struct dma_control_data *)0x20008000;
//__attribute__((aligned(1024)));

rtems_status_code mil1986be9x_dma_channel_obtain(unsigned channel)
{
	bool occupation = true;
	rtems_interrupt_level level;

	if (channel >= MIL1986BE9X_DMA_NUM)
		return RTEMS_INVALID_ID;

	rtems_interrupt_disable(level);
	occupation = dma_channel_occupation [channel];
	dma_channel_occupation [channel] = true;
	rtems_interrupt_enable(level);

	return occupation ? RTEMS_RESOURCE_IN_USE : RTEMS_SUCCESSFUL;
}

rtems_status_code mil1986be9x_dma_channel_release(unsigned channel)
{
	if (channel >= MIL1986BE9X_DMA_NUM)
		return RTEMS_INVALID_ID;

	dma_channel_occupation [channel] = false;
	return RTEMS_SUCCESSFUL;
}

static void (*dma_complete)(void *) = NULL;
static void *dma_complete_arg;

static void dma_interrupt_handler (void *arg)
{
	if (dma_complete)
	       dma_complete(dma_complete_arg);
	dma_complete = NULL;
}

#if 0
static void dump_buffer(uint8_t *buf, unsigned int len)
{
	int i;
	for (i =0; i < len; i++) {
		printk("%02x ", buf[i]);
		if ((i & 15) == 15)
			printk("\r\n");
	}
	printk("\r\n");
}

static void dump_buffer_32(uint32_t *buf, unsigned int len)
{
	int i;
	for (i =0; i < len; i++) {
		printk("%08x ", buf[i]);
		if ((i & 3) == 3)
			printk("\r\n");
	}
	printk("\r\n");
}

static void dump_buffer_16(uint16_t *buf, unsigned int len)
{
	int i;
	for (i =0; i < len; i++) {
		printk("%04x ", buf[i]);
		if ((i & 7) == 7)
			printk("\r\n");
	}
	printk("\r\n");
}
#endif

#define DMA_REPLASE_WINDOW(a)	\
	do {\
		uint32_t x = (uint32_t)(a); \
		x &= ~0xf0000000; \
		x |= 0x30000000; \
		(a) = (void *)x; \
	} while (0)

/*
 *
 */
static rtems_status_code dma_prepare(unsigned channel, void *dst,
			const void *src, size_t n,
			size_t dst_width, size_t src_width)
{
	uint32_t width;
	volatile struct dma_control_data *dc;

	if (channel > MIL1986BE9X_DMA_NUM)
		return RTEMS_INVALID_ID;

	dc = &dma_control[channel];

	DMA->CHNL_USEBURST_SET  = 1 << channel;
	DMA->CHNL_PRIORITY_SET  = 1 << channel;

	/* replace source and dest addres by memory address window accessed by DMA */
	DMA_REPLASE_WINDOW(src);
	DMA_REPLASE_WINDOW(dst);

	dst_width >>= 4;
	src_width >>= 4;
	width = (dst_width < src_width) ? dst_width : src_width;

	dc->control.value =
		DMA_CONTROL_SRC_SIZE(width) |
		DMA_CONTROL_DST_SIZE(width) |
		DMA_CONTROL_SRC_INC(src_width) |
		DMA_CONTROL_DST_INC(dst_width) |
		DMA_CONTROL_R_POWER(0xf) |
		DMA_CONTROL_N_MINUS_1(n - 1) |
		DMA_CONTROL_CYCLE_CTRL(2);

	dc->dst_end = (uint32_t)dst + ((n - 1) << (dst_width));
	dc->src_end = (uint32_t)src + ((n - 1) << (src_width));

	DMA->CHNL_ENABLE_SET    = 1 << channel;
	DMA->CHNL_REQ_MASK_CLR  = 1 << channel;
	DMA->CHNL_SW_REQUEST    = 1 << channel;
	return RTEMS_SUCCESSFUL;
}

/**
 *
 */
rtems_status_code mil1986be9x_dma_copy(unsigned channel, void *dst,
				       const void *src, size_t n,
				       size_t dst_width, size_t src_width)
{
	rtems_status_code sc;
	volatile struct dma_control_data *dc;

	sc = dma_prepare(channel, dst, src, n, dst_width, src_width);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	dc = &dma_control[channel];
	while (dc->control.bits.n_minus_1);
	return RTEMS_SUCCESSFUL;
}

/**
 *
 */
rtems_status_code mil1986be9x_dma_copy_async(unsigned channel, void *dst,
				       const void *src, size_t n,
				       size_t dst_width, size_t src_width,
				       void (*complete)(void *), void *arg)
{
	rtems_status_code sc;

	dma_complete = complete;
	dma_complete_arg = arg;
	sc = dma_prepare(channel, dst, src, n, dst_width, src_width);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	return RTEMS_SUCCESSFUL;
}

rtems_status_code mil1986be9x_dma_initialize(void)
{
	rtems_status_code sc = RTEMS_SUCCESSFUL;

	if (dma_initialized)
		return RTEMS_SUCCESSFUL;

	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_DMA;

	DMA->CFG |= DMA_CFG_MASTER_ENABLE;
	DMA->CHNL_ENABLE_CLR = 0xffffffff;
	DMA->CTRL_BASE_PTR = (uint32_t)dma_control;
	DMA->ALT_CTRL_BASE_PTR = (uint32_t)&dma_control[MIL1986BE9X_DMA_NUM];
	DMA->CHNL_REQ_MASK_SET = 0xffffffff;
	DMA->CHNL_PRIORITY_CLR = 0xffffffff;
	DMA->ERR_CLR = 1;

	/* Install interrupt handler */
	sc = rtems_interrupt_handler_install(
			DMA_IRQn,
			"DMA",
			RTEMS_INTERRUPT_UNIQUE,
			dma_interrupt_handler,
			NULL
			);
	RTEMS_CLEANUP_SC(sc, cleanup, "install interrupt handler");
	dma_initialized = true;

	printk("dma initialized\r\n");
	return RTEMS_SUCCESSFUL;

cleanup:
	return sc;
}

