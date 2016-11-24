/*
 *  console driver for mil1986be9x
 *
 *  Copyright (c) 2013
 *  Andrey Mitrofanov avmwww@gmail.com
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <bsp.h>
#include <bsp/start.h>
#include <bsp/1986be9x.h>
#include <rtems/bspIo.h>
#include <rtems/libio.h>
#include <rtems/termiostypes.h>
#include <termios.h>
#include <bsp/irq.h>

/*
 * Read one character from UART.
 *
 * Return -1 if there's no data, otherwise return
 * the character in lowest 8 bits of returned int.
 */
static int uart_read(int minor)
{
  UART_TypeDef *u;
  if (minor == 0)
    u = UART1;
  else if (minor == 1)
    u = UART2;
  else
    return -1;

  if (u->FR & UART_FR_RXFE)
    return -1;

  return u->DR & 0xFF;
}

/*
 * Write buffer to UART
 *
 * return 1 on success, -1 on error
 */
static ssize_t uart_write(int minor, const char *buf, size_t len)
{
  size_t i;
  UART_TypeDef *u;
  if (minor == 0)
    u = UART1;
  else if (minor == 1)
    u = UART2;
  else
    return -1;

  for (i = 0; i < len; i++) {
    while ((UART2->FR & UART_FR_TXFF) != 0) {}
    u->DR = (char)buf[i];
  }
  return 1;
}

static void uart_write_polled(int minor, char c)
{
  uart_write(minor, &c, 1);
}

int uart_poll_read(int minor)
{
  return uart_read(minor);
}

/*
 * Write a character to the console. This is used by printk() and
 * maybe other low level functions. It should not use interrupts or any
 * RTEMS system calls. It needs to be very simple
 */
static void _BSP_put_char( char c )
{
  uart_write_polled(1, c);
  if (c == '\n') {
    uart_write_polled(1, '\r');
  }
}
BSP_output_char_function_type BSP_output_char = _BSP_put_char;

static int _BSP_get_char(void)
{
  return uart_poll_read(1);
}

BSP_polling_getchar_function_type BSP_poll_char = _BSP_get_char;

void console_outbyte_polled(
  int  port,
  char c
)
{
  _BSP_put_char(c);
}

int console_inbyte_nonblocking(
  int port
)
{
  return _BSP_get_char();
}

/*
 *  console_initialize_hardware
 *
 *  This routine initializes the console hardware.
 *
 */
void console_initialize_hardware(void)
{
  return;
}


