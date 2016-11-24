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

#include <bsp.h>
#include <bsp/start.h>
#include <rtems/score/armv7m.h>

#define SYSCLOCK_MULTIPLIER (MIL1986BE9X_SYSTEM_CLOCK/MIL1986BE9X_INPUT_CLOCK)

#include <bsp/1986be9x.h>

/* Setup external RAM */
static void inline BSP_START_TEXT_SECTION ext_ram_init(void)
{
    RST_CLK->PER_CLOCK |= \
                          RST_CLK_PER_CLOCK_PORTA | \
                          RST_CLK_PER_CLOCK_PORTB | \
                          RST_CLK_PER_CLOCK_PORTC | \
                          RST_CLK_PER_CLOCK_PORTD | \
                          RST_CLK_PER_CLOCK_PORTE | \
                          RST_CLK_PER_CLOCK_PORTF; // Clock periferial devs
    // Set funct and power, digital
#define SETUP_PORT(PORT,F,A,P) (PORT->FUNC = F, PORT->ANALOG = A, PORT->PWR = P)
#define SETUP_PORT_MAIN(PORT) SETUP_PORT(PORT,0x55555555,0xFFFF,0xFFFFFFFF)
    SETUP_PORT_MAIN(PORTA);
    SETUP_PORT_MAIN(PORTB);
    SETUP_PORT(PORTC, 0xAA001554, 0xFC7E, 0xF0F03FFC);
    PORTC->OE = 0x0C00;
    PORTC->RXTX = 0x0000;
    SETUP_PORT(PORTD, 0xC3FFE800, 0x9FFF, 0x03FFFC00);
    SETUP_PORT(PORTE, 0x55555555, 0xFFFF, 0xFFFFFFFF);
    SETUP_PORT(PORTF, 0x5555555F, 0xFFFF, 0xFFFFFFFF);

    RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_EXT_BUS;
    EXT_BUS_CNTRL->EXT_BUS_CONTROL = EXT_BUS_CNTRL_EXT_BUS_CONTROL_RAM | \
                                     (0xF << EXT_BUS_CNTRL_EXT_BUS_CONTROL_WAIT_STATE_Pos);
}

static void inline BSP_START_TEXT_SECTION benchmark_timers_init()
{
    RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_TIMER1|RST_CLK_PER_CLOCK_TIMER2;
    RST_CLK->TIM_CLOCK |= RST_CLK_TIM_CLOCK_TIM1_CLK_EN|RST_CLK_TIM_CLOCK_TIM2_CLK_EN;

    TIMER1->CNT = 0;
    TIMER1->ARR = 0xFFFF;
    TIMER1->PSG = 0;
    TIMER1->CNTRL = TIMER_CNTRL_CNT_EN;

    TIMER2->CNT = 0;
    TIMER2->ARR = 0xFFFF;
    TIMER2->PSG = 0;
    TIMER2->CNTRL = (0b0001 << 8) | (0b10 << 6) | TIMER_CNTRL_CNT_EN;
}

void BSP_START_TEXT_SECTION bsp_start_hook_0(void)
{
    /* clear primask bit */
    _ARMV7M_Set_primask(0);
    /* Enable HSE generator. */
    RST_CLK->HS_CONTROL = RST_CLK_HS_CONTROL_HSE_ON;
    while (! (RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY))
        continue;

    /* Set up EEPROM waitstates */
#define SET_EEPROM_DELAY(WS) (EEPROM->CMD=((EEPROM->CMD)&(~EEPROM_CMD_Delay_Msk))|(WS<<EEPROM_CMD_Delay_Pos))
#if (MIL1986BE9X_SYSTEM_CLOCK < 25000000)
    SET_EEPROM_DELAY(0);
#elif (MIL1986BE9X_SYSTEM_CLOCK < 50000000)
    SET_EEPROM_DELAY(1);
#elif (MIL1986BE9X_SYSTEM_CLOCK < 75000000)
    SET_EEPROM_DELAY(2);
#endif

    ext_ram_init();

    /* Enable clock to watchdog timer */
    RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_IWDT;

    benchmark_timers_init();

    /* Use HSE for CPU_C1 clock. */
    RST_CLK->CPU_CLOCK = (2<<RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos);

    /* Setup PLL for CPU. */
    RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos);
    RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) |
        RST_CLK_PLL_CONTROL_PLL_CPU_ON;
    RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) |
        RST_CLK_PLL_CONTROL_PLL_CPU_ON | RST_CLK_PLL_CONTROL_PLL_USB_RLD;
    RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) |
        RST_CLK_PLL_CONTROL_PLL_CPU_ON;

    while (!(RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY));

    /* Use PLLCPUo for CPU_C2, CPU_C3 and HCLK. */
    RST_CLK->CPU_CLOCK = (1<<RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos) |
        (2<<RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos) |
        (1<<RST_CLK_CPU_CLOCK_HCLK_SEL_Pos);

    /* Set UART2 for debug output. */
    RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PORTF; // вкл. тактирования PORTF
    PORTF->FUNC |= 0b1111; // переопределенная функция для
    // PF0(UART2_RXD) и PF1(UART2_TXD)
    PORTF->ANALOG |= 3; // цифровые выводы
    PORTF->PWR &= ~0b1111;
    PORTF->PWR |= 0b0101;

    RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_UART2;// вкл. тактирования UART2

    /* Set baud rate divisor: 115200 bit/sec. */
    RST_CLK->UART_CLOCK = RST_CLK_UART_CLOCK_UART2_CLK_EN | // разрешаем тактирование UART2
        (0<<RST_CLK_UART_CLOCK_UART2_BRG_Pos);// HCLK (8 МГц)
    UART2->IBRD = (MIL1986BE9X_SYSTEM_CLOCK/MIL1986BE9X_UART_BAUD/16);
    UART2->FBRD = ((MIL1986BE9X_SYSTEM_CLOCK*4/MIL1986BE9X_UART_BAUD)&077);

    /* Enable UART2, transmiter only, 8 bit. */
    UART2->LCR_H = (3 << UART_LCR_H_WLEN_Pos);
    UART2->CR = UART_CR_UARTEN | UART_CR_RXE | UART_CR_TXE;
}

void BSP_START_TEXT_SECTION bsp_start_hook_1(void)
{
    bsp_start_copy_sections();
    bsp_start_clear_bss();

    /* At this point we can use objects outside the .start section */
}

