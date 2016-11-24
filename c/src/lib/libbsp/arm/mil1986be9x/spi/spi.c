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
#include <rtems/libio.h>
#include <rtems/error.h>
#include <stdio.h>

#define RTEMS_STATUS_CHECKS_USE_PRINTK
#include <rtems/status-checks.h>

#include <bsp.h>
#include <bsp/irq-generic.h>
#include <bsp/1986be9x.h>
#include <bsp/spi.h>


//#define DEBUG
//#define DEBUG_IRQ

#define SPI_NUM			2
/*
 * Queue of SPI packets.
 */
#define SPI_QUEUE_SIZE          4096      /* max number of packets in queue */

typedef struct _spi_queue_t {
  unsigned int in;        /* data is added at offset (in % size) */
  unsigned int out;       /* data is extracted from off. (out % size) */
  unsigned char buffer[SPI_QUEUE_SIZE];
} spi_queue_t;

#define min(a, b)	(((a) < (b)) ? (a) : (b))

struct spi_port {
  unsigned                      port;           /* port number */
  unsigned                      irq;            /* interrupt number */
  unsigned                      master;         /* master or slave */
  unsigned                      kbps;           /* kbits per second */
  int                           inited;
  int                           opened;
  rtems_device_major_number     major;
  rtems_device_minor_number     minor;
  char                          devname[16];
  volatile spi_queue_t		inq;            /* queue of received packets */

  rtems_id                      sem;
  rtems_id                      irq_sem;
  /* Wait timeout */
  uint32_t			timeout;
  uint32_t			flags;

  /* Statistics. */
  unsigned long interrupts;	/* interrupt counter */
  unsigned long out_packets;	/* transmitted packets */
  unsigned long in_packets;	/* received packets */
  unsigned long in_discards;	/* ignored packets, due to lack of memory */
};
static struct spi_port spi_port[SPI_NUM];

/*
 * Initialize queue.
 */
static inline __attribute__((always_inline))
void spi_queue_init (spi_queue_t *q)
{
	q->in = q->out = 0;
}

/*
 * Add a packet to queue.
 */
static inline __attribute__((always_inline))
unsigned int spi_queue_put (spi_queue_t *q,
			    unsigned char *buffer, unsigned int len)
{
	unsigned int l;

	len = min(len, SPI_QUEUE_SIZE - q->in + q->out);
	l = min(len, SPI_QUEUE_SIZE - (q->in & (SPI_QUEUE_SIZE - 1)));
	memcpy(q->buffer + (q->in & (SPI_QUEUE_SIZE - 1)), buffer, l);
	memcpy(q->buffer, buffer + l, len - l);
	q->in += len;

	return len;
}

/*
 * Get a packet from queue.
 */
static inline __attribute__((always_inline))
unsigned int spi_queue_get (spi_queue_t *q,
			      unsigned char *buffer, unsigned int len)
{
	unsigned int l;

	len = min(len, q->in - q->out);
	l = min(len, SPI_QUEUE_SIZE - (q->out & (SPI_QUEUE_SIZE - 1)));
	memcpy(buffer, q->buffer + (q->out & (SPI_QUEUE_SIZE - 1)), l);
	memcpy(buffer + l, q->buffer, len - l);
	q->out += len;

	if (q->in == q->out)
		q->in = q->out = 0;
	return len;
}

/*
 * Get len in bytes in queue
 */
static inline __attribute__((always_inline))
int spi_queue_len (spi_queue_t *q)
{
	return q->in - q->out;
}

/*
 * Инициализация внешних сигналов SSP1.
 */
static void spi_setup_ssp1 ()
{
	uint32_t val;
	/* Включаем тактирование порта SSP1. */
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_SSP1;

	/* Разрешение тактовой частоты на SSP1, источник HCLK. */
	val = RST_CLK->SSP_CLOCK & ~RST_CLK_SSP_CLOCK_SSP1_BRG_Msk;
	val |= RST_CLK_SSP_CLOCK_SSP1_CLK_EN |
	       ((0 << RST_CLK_SSP_CLOCK_SSP1_BRG_Pos) & RST_CLK_SSP_CLOCK_SSP1_BRG_Msk);
	RST_CLK->SSP_CLOCK = val;
}

/*
 * Инициализация внешних сигналов SSP2.
 */
static void spi_setup_ssp2 ()
{
	uint32_t val;
	/* Включаем тактирование порта SSP2. */
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_SSP2;

	/* Разрешение тактовой частоты на SSP2, источник HCLK. */
	val = RST_CLK->SSP_CLOCK & ~RST_CLK_SSP_CLOCK_SSP2_BRG_Msk;
	val |= RST_CLK_SSP_CLOCK_SSP2_CLK_EN |
	       ((0 << RST_CLK_SSP_CLOCK_SSP2_BRG_Pos) & RST_CLK_SSP_CLOCK_SSP2_BRG_Msk);
	RST_CLK->SSP_CLOCK = val;
}

static int spi_output_block (struct spi_port *c, char *data, int count, int *tx_count)
{
	SPI_TypeDef *reg = (c->port == 0) ? SPI1 : SPI2;
	rtems_status_code sc;
	int cnt = 0;

	sc = rtems_semaphore_obtain (c->sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	while (count) {
		if (! (reg->SSPx_SR & SPI_SSPx_SR_TNF)) {
#ifdef DEBUG
			printk("Busy tx\r\n");
#endif
			if (c->flags & LIBIO_FLAGS_NO_DELAY)
				break;
			/* Ждём появления места в FIFO передатчика. */
			sc = rtems_semaphore_obtain (c->irq_sem, RTEMS_WAIT, c->timeout);
			if (sc != RTEMS_SUCCESSFUL)
				break;
#ifdef DEBUG
			printk("ready tx\r\n");
#endif
		} else {
			reg->SSPx_DR = *data++;
			c->out_packets++;
			cnt++;
			count--;
		}
	}
	rtems_semaphore_release (c->sem);
	*tx_count = cnt;
	return RTEMS_SUCCESSFUL;
}

/*
 * Извлекаем данные из приёмного FIFO.
 */
static int receive_data (struct spi_port *c)
{
	SPI_TypeDef *reg = (c->port == 0) ? SPI1 : SPI2;
	int nwords = 0;
	unsigned char spi_data[16];
	int i;

	while (reg->SSPx_SR & SPI_SSPx_SR_RNE) {
		rtems_interrupt_level v;
		i = 0;
		while ((reg->SSPx_SR & SPI_SSPx_SR_RNE) && i < sizeof(spi_data)) {
			spi_data[i++] = reg->SSPx_DR;
			nwords++;
		}

		rtems_interrupt_disable(v);
		spi_queue_put (&c->inq, spi_data, i);
		rtems_interrupt_enable(v);
	}
	c->in_packets += nwords;
	return nwords;
}

/*
 * Wait for word received.
 */
static int spi_input_block (struct spi_port *c, unsigned char *data, int count, int *rx_count)
{
	rtems_status_code sc;
	int cnt = 0;

	sc = rtems_semaphore_obtain (c->sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	while (count) {
		if (spi_queue_len (&c->inq) == 0) {
			if (c->flags & LIBIO_FLAGS_NO_DELAY)
				break;
			/* Ждём приёма пакета. */
			sc = rtems_semaphore_obtain (c->irq_sem, RTEMS_WAIT, c->timeout);
			if (sc != RTEMS_SUCCESSFUL)
				break;
		} else {
			unsigned int n;
			rtems_interrupt_level v;

			rtems_interrupt_disable(v);
			n = spi_queue_get(&c->inq, data, count);
			rtems_interrupt_enable(v);
			data += n;
			count -= n;
			cnt += n;
		}
	}
	rtems_semaphore_release (c->sem);
	*rx_count = cnt;
	return sc;
}

/*
 * Set up the SPI driver.
 */
static void spi_init (struct spi_port *c, int bits_per_word,
                      unsigned nsec_per_bit, unsigned mode)
{
	SPI_TypeDef *reg = (c->port == 0) ? SPI1 : SPI2;
	int proto = 0; /* Motorolla spi protocol TODO: change by ioctl */
	unsigned int ssp_clk;
	unsigned int div_tab[] = {1,2,4,8,16,32,64,128};

	/* Инициализация структуры данных драйвера. */
	c->master = (nsec_per_bit > 0);
	spi_queue_init (&c->inq);

	/* Выбор соответствующего интерфейса SSP и
	 * установка внешних сигналов. */
	if (c->port == 0) {
		spi_setup_ssp1 ();
		reg = SPI1;
		c->irq = SSP1_IRQn;
	} else {
		spi_setup_ssp2 ();
		reg = SPI2;
		c->irq = SSP2_IRQn;
	}

	/* Инициализация всех регистров данного интерфейса SSP.
	 * Ловим прерывания от приёмника. */
	reg->SSPx_CR0 = ((proto << SPI_SSPx_CR0_FRF_Pos) & SPI_SSPx_CR0_FRF_Msk) |
			(((bits_per_word - 1) << SPI_SSPx_CR0_DSS_Pos) & SPI_SSPx_CR0_DSS_Msk) |
			SPI_SSPx_CR0_SPH |
			mode;

	if (c->port == 0)
	       	ssp_clk = ((RST_CLK->SSP_CLOCK & RST_CLK_SSP_CLOCK_SSP1_BRG_Msk) >> RST_CLK_SSP_CLOCK_SSP1_BRG_Pos);
	else
	       	ssp_clk = ((RST_CLK->SSP_CLOCK & RST_CLK_SSP_CLOCK_SSP2_BRG_Msk) >> RST_CLK_SSP_CLOCK_SSP2_BRG_Pos);
	ssp_clk = MIL1986BE9X_SYSTEM_CLOCK / div_tab[ssp_clk];

	if (c->master) {
		/* Режим master. */
		unsigned divisor = ((ssp_clk /1000) * nsec_per_bit + 1999999) / 2000000;
		reg->SSPx_CR0 |= ((divisor - 1)  << SPI_SSPx_CR0_SCR_Pos) & SPI_SSPx_CR0_SCR_Msk;
		reg->SSPx_CR1 = 0;
		reg->SSPx_CPSR = 2;
		c->kbps = (ssp_clk / ((divisor + 1) * 2)) / 1000;
#ifdef DEBUG
		printk("spi %d speed KHZ = %d divisor %d\r\n",
			c->port, MIL1986BE9X_SYSTEM_CLOCK/1000, divisor);
#endif
	} else {
		/* Режим slave.
		 * Максимальная частота равна MIL1986BE9X_SYSTEM_CLOCK/12. */
		reg->SSPx_CR1 = SPI_SSPx_CR1_MS;
		reg->SSPx_CPSR = 12;
		c->kbps = ((ssp_clk / 1000) + 6) / 12;
	}
	reg->SSPx_DMACR = 0;
	reg->SSPx_IMSC = SPI_SSPx_IMSC_RXIM | SPI_SSPx_IMSC_RTIM;
	reg->SSPx_CR1 |= SPI_SSPx_CR1_SSE;
#ifdef DEBUG
	{

	printk("HCLK %u Hz, SSP%d_CLK %u Hz, spi speed %u Hz\r\n",
		MIL1986BE9X_SYSTEM_CLOCK, c->port + 1, ssp_clk,
		ssp_clk/
		(reg->SSPx_CPSR * (1 + ((reg->SSPx_CR0 & SPI_SSPx_CR0_SCR_Msk) >> SPI_SSPx_CR0_SCR_Pos))));
	}
#endif
}

static rtems_isr spi_irq_handler(void *arg)
{
	struct spi_port *c = (struct spi_port *)arg;
	SPI_TypeDef *reg = (c->port == 0) ? SPI1 : SPI2;

#ifdef DEBUG_IRQ
	printk("IRQ spi%d\r\n", c->port);
#endif
	c->interrupts++;
	receive_data (c);
	rtems_semaphore_release (c->irq_sem);
}


static struct spi_port *spi_get_port(
		rtems_device_major_number  major,
		rtems_device_minor_number  minor
		)
{
	if (major == spi_port[0].major)
		return &spi_port[0];
	else if (major == spi_port[1].major)
		return &spi_port[1];
	else
		return NULL;
}

static rtems_device_driver drv_spi_initialize(
		rtems_device_major_number  major,
		rtems_device_minor_number  minor,
		void                      *arg
		)
{
	rtems_status_code status;
	char irq_name[] = "SPI0";
	struct spi_port *c = spi_get_port(major, minor);

	if (!c)
		return RTEMS_UNSATISFIED;

	irq_name[3] = c->port + '0';
	/*
	 * HW initialize
	 */
	/* SPI0 = master, SPI1 = slave, 1 MHz */
	spi_init(c, 8, c->port ? 0 : 1000, 0);

	status = rtems_semaphore_create (
			rtems_build_name ('S', 'P', 'S', c->port + '0'),
			1,
			RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
			RTEMS_NO_PRIORITY,
			&c->sem);
	if (status != RTEMS_SUCCESSFUL)
		rtems_fatal_error_occurred (status);

	/*
	 * create semaphore for IRQ synchronization
	 */
	status = rtems_semaphore_create(
			rtems_build_name('S','P','I',c->port + '0'),
			0,
			RTEMS_FIFO | RTEMS_SIMPLE_BINARY_SEMAPHORE,
			RTEMS_NO_PRIORITY,
			&c->irq_sem);
	if (status != RTEMS_SUCCESSFUL)
		rtems_fatal_error_occurred (status);

	/*
	 * Connect to IRQ
	 */
	status = rtems_interrupt_handler_install(
			c->irq,
			irq_name,
			RTEMS_INTERRUPT_UNIQUE,
			spi_irq_handler,
			(void *)c);

	RTEMS_CHECK_SC(status, "install interrupt handler\r");

	/*
	 * Register the device
	 */
	sprintf(c->devname, "/dev/spi%d", c->port);
	status = rtems_io_register_name(c->devname, major, minor);
	if (status != RTEMS_SUCCESSFUL)
		rtems_fatal_error_occurred(status);

	c->opened = FALSE;
#ifdef DEBUG
	printk("Device: %s registered.\r\n", c->devname);
#endif
	return RTEMS_SUCCESSFUL;
}

/*
 *  Open entry point
 */
static rtems_device_driver drv_spi_open(
		rtems_device_major_number major,
		rtems_device_minor_number minor,
		void                    * arg
		)
{
	rtems_status_code sc;
	struct spi_port *c;
	SPI_TypeDef *reg;
	rtems_libio_open_close_args_t *a = arg;

	if (major == spi_port[0].major)
		c = &spi_port[0];
	else if (major == spi_port[1].major)
		c = &spi_port[1];
	else
		return RTEMS_UNSATISFIED;

	sc = rtems_semaphore_obtain (c->sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	if (c->opened) {
		rtems_semaphore_release (c->sem);
		return RTEMS_RESOURCE_IN_USE;
	}

	c->opened = TRUE;
	c->timeout = RTEMS_NO_TIMEOUT;
	c->flags = a->flags;
	reg = (c->port == 0) ? SPI1 : SPI2;
	reg->SSPx_CR1 |= SPI_SSPx_CR1_SSE;
	rtems_semaphore_release (c->sem);
	return RTEMS_SUCCESSFUL;
}

/*
 *  Close entry point
 */
static rtems_device_driver drv_spi_close(
		rtems_device_major_number major,
		rtems_device_minor_number minor,
		void                    * arg
		)
{
	rtems_status_code sc;
	struct spi_port *c;
	SPI_TypeDef *reg;

	c = spi_get_port(major, minor);
	if (!c)
		return RTEMS_UNSATISFIED;

	sc = rtems_semaphore_obtain (c->sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	c->opened = FALSE;
	reg = (c->port == 0) ? SPI1 : SPI2;
	reg->SSPx_CR1 &= ~SPI_SSPx_CR1_SSE;
	rtems_semaphore_release (c->sem);
	return RTEMS_SUCCESSFUL;
}

/*
 *
 */
static rtems_device_driver drv_spi_read(
		rtems_device_major_number major,
		rtems_device_minor_number minor,
		void                    * arg
		)
{
	rtems_libio_rw_args_t *rw = arg;
	rtems_status_code sc;
	struct spi_port *c;
	int count = 0;

	c = spi_get_port(major, minor);
	if (!c)
		return RTEMS_UNSATISFIED;

	sc = spi_input_block (c, (unsigned char *)rw->buffer, rw->count, &count);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	rw->bytes_moved = count;
	return RTEMS_SUCCESSFUL;
}

/*
 *
 */
static rtems_device_driver drv_spi_write(
		rtems_device_major_number major,
		rtems_device_minor_number minor,
		void                    * arg
		)
{
	rtems_libio_rw_args_t *rw = (rtems_libio_rw_args_t *)arg;
	rtems_status_code sc;
	struct spi_port *c;
	int count = 0;

	c = spi_get_port(major, minor);
	if (!c)
		return RTEMS_UNSATISFIED;

	sc = spi_output_block (c, rw->buffer, rw->count, &count);
	if (sc != RTEMS_SUCCESSFUL)
		return sc;

	rw->bytes_moved = count;
	return RTEMS_SUCCESSFUL;
}

/*
 *
 */
static rtems_device_driver drv_spi_control(
		rtems_device_major_number major,
		rtems_device_minor_number minor,
		void                    * arg
		)
{
	rtems_libio_ioctl_args_t *args = arg;
	struct spi_port *c;

	c = spi_get_port(major, minor);
	if (!c)
		return RTEMS_UNSATISFIED;

	switch (args->command) {
	case SPI_IOCTL_SET_TIMEOUT:
		c->timeout = *((uint32_t *) args->buffer);
		args->ioctl_return = 0;
		break;
	default:
		args->ioctl_return = -1;
		return RTEMS_INVALID_ID;
	}
	return RTEMS_SUCCESSFUL;
}

static rtems_driver_address_table drv_spi = {
	initialization_entry:	drv_spi_initialize,
	open_entry:		drv_spi_open,
	close_entry:		drv_spi_close,
	read_entry:		drv_spi_read,
	write_entry:		drv_spi_write,
	control_entry:		drv_spi_control,
};

static int spi_do_finalize(struct spi_port *spi)
{
	rtems_status_code status;

	if (!spi->inited)
		return RTEMS_SUCCESSFUL;

	if (spi->opened) {
		printk("SPI%d device is opened, unable to proceed\r\n", spi->port);
		return RTEMS_RESOURCE_IN_USE;
	}
	status = rtems_io_unregister_driver(spi->major);
	if (status != RTEMS_SUCCESSFUL) {
		printk("Unable to remove SPI%d driver\r\n", spi->port);
		return status;
	}
	status = (rtems_status_code)unlink(spi->devname);
	if (status != RTEMS_SUCCESSFUL) {
		printk("Unable to remove SPI%d device node from file system\r\n", spi->port);
	};

	rtems_semaphore_delete(spi->sem);
	rtems_semaphore_delete(spi->irq_sem);
	printk("SPI%d driver unloaded successfully\r\n", spi->port);
	spi->inited=FALSE;
	return RTEMS_SUCCESSFUL;
}

static int spi_do_initialize(struct spi_port *spi)
{
	if (!spi->inited) {
		if (RTEMS_SUCCESSFUL==rtems_io_register_driver(0, &drv_spi, &spi->major))
			spi->inited=TRUE;
	}
	return spi->inited;
}

int spi_initialize(void)
{
	int port;
	for (port = 0; port < SPI_NUM; port++) {
		spi_port[port].port = port;
		if (!spi_do_initialize(&spi_port[port])) {
			if (port)
				spi_do_finalize(&spi_port[0]);
			return RTEMS_UNSATISFIED;
		}
	}
	return RTEMS_SUCCESSFUL;
}

int spi_finalize(void)
{
	int port;
	int err;
	for (port = 0; port < SPI_NUM; port++) {
		if ((err = spi_do_finalize(&spi_port[port]))!= RTEMS_SUCCESSFUL)
			return err;
	}
	return RTEMS_SUCCESSFUL;
}
