/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/


// Smart Valve

#include "global.h"

#include <tmr.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "spim.h"
#include "uart.h"

#include "pmu.h"

#include "pwrseq_regs.h"
#include "pwrman_regs.h"
#include "flc.h"
#include "rtc_regs.h"
#include "trim_regs.h"
#include "board.h"
#include "crc.h"
#include "rtc.h"
#include "lp.h"

#include "flow.h"

#include "tdc.h"

#define RTC_HZ	4096


#define INTERRUPT_PRIORITY_DEFAULT			(configMAX_SYSCALL_INTERRUPT_PRIORITY-2)
#define INTERRUPT_PRIORITY_FREERTOS_TIMER	(configMAX_SYSCALL_INTERRUPT_PRIORITY-1)

static uart_write_cb_t s_p_uart_write_cb;

const gpio_cfg_t s_gpio_cfg_led[] =
{
    { PORT_3, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL },
    { PORT_3, PIN_1, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL },
    { PORT_3, PIN_2, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL },
    { PORT_3, PIN_3, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL }
};

static const gpio_cfg_t s_gpio_cfg_button_s1 = { PORT_2, PIN_3, GPIO_FUNC_GPIO, GPIO_PAD_INPUT };
static const gpio_cfg_t s_gpio_cfg_button_s2 = { PORT_2, PIN_2, GPIO_FUNC_GPIO, GPIO_PAD_INPUT };

typedef struct _board_switch_t
{
    bool		state;
    bool		changed;
    uint8_t		debounce_count;
    const gpio_cfg_t * p_gpio_cfg;
}
board_switch_t;

static board_switch_t s_switches[2] =
{
    { .p_gpio_cfg = &s_gpio_cfg_button_s1 },
    { .p_gpio_cfg = &s_gpio_cfg_button_s2 }
};

static const gpio_cfg_t s_tdc_interrupt = { PORT_0, PIN_5, GPIO_FUNC_GPIO, GPIO_PAD_INPUT_PULLUP };

static const ioman_cfg_t spi_cfg = IOMAN_SPIM1( 1, 1, 0, 0, 0, 0 );
static const spim_cfg_t max3510x_spim_cfg = { 1, SPIM_SSEL0_LOW, 12000000 };
static const gpio_cfg_t max3510x_spi = { PORT_1, (PIN_0 | PIN_1 | PIN_2 | PIN_3), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

static const ioman_cfg_t g_uart_cfg = IOMAN_UART( UART_NDX, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0 );
static const gpio_cfg_t max3510x_uart = { PORT_0, (PIN_0 | PIN_1), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };





void GPIO_P0_IRQHandler( void )
{
    GPIO_Handler( 0 );
}

void PMU_IRQHandler( void )
{
    PMU_Handler();
}

void UART1_IRQHandler( void )
{
    UART_Handler( BOARD_UART );
}

void vApplicationIdleHook( void )
{
    LP_EnterLP2();
}

#ifdef ENABLE_LP1_IDLE

void RTC0_IRQHandler( void )
{
    RTC_ClearFlags( MXC_F_RTC_FLAGS_COMP0 );
    flow_sample_clock();
}

void RTC1_IRQHandler( void )
{
    RTC_ClearFlags( MXC_F_RTC_FLAGS_SNOOZE );
    xPortSysTickHandler();
}


void vPortSetupTimerInterrupt( void )
{
    LP_ClearWakeUpConfig();
    LP_ConfigRTCWakeUp( 1, 1, 0, 0 );  // enable RTC COMP1 & COMP0 wake-ups
    {
        rtc_cfg_t cfg =
        {
            .snoozeMode = RTC_SNOOZE_MODE_A,
            .snoozeCount = RTC_HZ / configTICK_RATE_HZ,
            .compareCount[0] = RTC_HZ / FLOW_SAMPLE_CLOCK_HZ;
        };
        if( RTC_Init( &cfg ) != E_NO_ERROR )
            while( 1 );
        RTC_EnableINT( MXC_F_RTC_INTEN_COMP1 | MXC_F_RTC_INTEN_COMP0 );
        NVIC_SetPriority( RTC1_IRQn, INTERRUPT_PRIORITY_FREERTOS_TIMER );
        NVIC_EnableIRQ( RTC1_IRQn );
        RTC_Start();
    }
}

#else // ENABLE_LP1_IDLE

void TMR1_IRQHandler( void )
{
    TMR32_ClearFlag( MXC_TMR1 );
    flow_sample_clock();

}

#endif // ENABLE_LP1_IDLE


void board_init( void )
{
    // brings up board-specific ports

    SYS_IOMAN_UseVDDIOH( &max3510x_spi );
    SYS_IOMAN_UseVDDIOH( &s_tdc_interrupt );

    uint8_t i;
    for( i = 0; i < ARRAY_COUNT( s_gpio_cfg_led ); i++ )
    {
        SYS_IOMAN_UseVDDIOH( &s_gpio_cfg_led[i] );
        board_led( i, false );
        GPIO_Config( &s_gpio_cfg_led[i] );
    }
    GPIO_Config( &s_tdc_interrupt );
    GPIO_Config( &max3510x_spi );
    GPIO_Config( &max3510x_uart );
    GPIO_Config( &s_gpio_cfg_button_s1 );
    GPIO_Config( &s_gpio_cfg_button_s2 );

    FLC_Init();


    {
        // initialize the SPI port connected to the MAX35103
        sys_cfg_t sys_cfg;
        sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;
        sys_cfg.io_cfg = spi_cfg;
        if( SPIM_Init( BOARD_TDC_SPI, &max3510x_spim_cfg, &sys_cfg ) != E_NO_ERROR )
        {
            while( 1 ); // initialization failed -- step into CSL to determine the reason
        }
        BOARD_TDC_SPI->mstr_cfg = (BOARD_TDC_SPI->mstr_cfg & ~MXC_F_SPIM_MSTR_CFG_PAGE_SIZE) | MXC_S_SPIM_MSTR_CFG_PAGE_4B;
//		BOARD_TDC_SPI->fifo_ctrl = (BOARD_TDC_SPI->fifo_ctrl & ~(MXC_F_SPIM_FIFO_CTRL_RX_FIFO_AF_LVL|MXC_F_SPIM_FIFO_CTRL_TX_FIFO_AE_LVL))
//			| (BOARD_SPI_RX_TRIGGER_LEVEL << MXC_F_SPIM_FIFO_CTRL_RX_FIFO_AF_LVL_POS) | (BOARD_SPI_TX_TRIGGER_LEVEL << MXC_F_SPIM_FIFO_CTRL_TX_FIFO_AE_LVL_POS);
    }

#ifndef ENABLE_LP1_IDLE
    {
        // Timer 1 provides the flow sample clock when operating in LP2 mode
        tmr32_cfg_t cont_cfg;
        cont_cfg.mode = TMR32_MODE_CONTINUOUS;
        cont_cfg.polarity = TMR_POLARITY_INIT_LOW;  //start GPIO low
        cont_cfg.compareCount = configCPU_CLOCK_HZ / FLOW_SAMPLE_CLOCK_HZ;
        while( TMR_Init( MXC_TMR1, TMR_PRESCALE_DIV_2_0, NULL ) != E_NO_ERROR );
        TMR32_Config( MXC_TMR1, &cont_cfg );
    }
    NVIC_SetPriority( TMR1_0_IRQn, INTERRUPT_PRIORITY_DEFAULT );
    TMR32_EnableINT( MXC_TMR1 );
    NVIC_EnableIRQ( TMR1_0_IRQn );
#endif
    {
        // initialize the UART connected to the PICO USB serial port
        uart_cfg_t cfg;
        cfg.parity = UART_PARITY_DISABLE;
        cfg.size = UART_DATA_SIZE_8_BITS;
        cfg.extra_stop = 0;
        cfg.cts = 0;
        cfg.rts = 0;
        cfg.baud = 115200;

        sys_cfg_uart_t sys_cfg;
        sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;
        sys_cfg.io_cfg = g_uart_cfg;

        while( UART_Init( BOARD_UART, &cfg, &sys_cfg ) != E_NO_ERROR );
    }
    NVIC_SetPriority( UART_IRQ, INTERRUPT_PRIORITY_DEFAULT );
    board_uart_enable_interrupt();

    NVIC_SetPriority( GPIO_P0_IRQn, INTERRUPT_PRIORITY_DEFAULT );
    GPIO_IntConfig( &s_tdc_interrupt, GPIO_INT_FALLING_EDGE );
    GPIO_RegisterCallback( &s_tdc_interrupt, tdc_interrupt, NULL );
    board_tdc_enable_interrupt();

    BOARD_UART->tx_fifo_ctrl = BOARD_UART_TX_FIFO_LVL << MXC_F_UART_TX_FIFO_CTRL_FIFO_AE_LVL_POS;

    NVIC_SetPriority( PMU_IRQn, INTERRUPT_PRIORITY_DEFAULT );
    NVIC_EnableIRQ( PMU_IRQn );
}

void board_uart_enable_interrupt( void )
{
    NVIC_EnableIRQ( UART_IRQ );
}

void board_uart_disable_interrupt( void )
{
    NVIC_DisableIRQ( UART_IRQ );
}

void board_tdc_enable_interrupt( void )
{
    GPIO_IntEnable( &s_tdc_interrupt );
}

void board_tdc_disable_interrupt( void )
{
    GPIO_IntDisable( &s_tdc_interrupt );
}

void max3510x_spi_xfer( max3510x_t p, void * pv_in, const void * pv_out, uint8_t count )
{
    // used by the MAX3510x module to interface with the hardware

//	tdc_shield_t *p_shield = (tdc_shield_t *)p;
    spim_req_t req;
    req.ssel = 0;
    req.deass = 1;
    req.tx_data = pv_out;
    req.rx_data = pv_in;
    req.width = SPIM_WIDTH_1;
    req.len = count;

    if( (SPIM_Trans( BOARD_TDC_SPI, &req )) != count )
    {
        while( 1 ); // fatal error -- step into CSL to determine reason
    }

    // Wait for transaction to complete
    while( SPIM_Busy( BOARD_TDC_SPI ) != E_NO_ERROR )
    {
        // fatal
    }
}

bool board_flash_write( const void * p_data, uint16_t size )
{
    // The last 8K page in flash is reserved for max3510x configuration
    if( size <= MXC_FLASH_PAGE_SIZE )
    {
        if( E_NO_ERROR == FLC_PageErase( (MXC_FLASH_MEM_BASE + MXC_FLASH_FULL_MEM_SIZE - MXC_FLASH_PAGE_SIZE), MXC_V_FLC_ERASE_CODE_PAGE_ERASE, MXC_V_FLC_FLSH_UNLOCK_KEY ) )
            if( E_NO_ERROR == FLC_Write( (MXC_FLASH_MEM_BASE + MXC_FLASH_FULL_MEM_SIZE - MXC_FLASH_PAGE_SIZE), p_data, size + (((4 - size) & 3) & 3), MXC_V_FLC_FLSH_UNLOCK_KEY ) )
                return true;
    }
    return false;
}

void board_flash_read( void * p_data, uint16_t size )
{
    if( size <= MXC_FLASH_PAGE_SIZE )
    {
        memcpy( p_data, (void*)(MXC_FLASH_MEM_BASE + MXC_FLASH_FULL_MEM_SIZE - MXC_FLASH_PAGE_SIZE), size );
    }
}


void board_led( uint8_t ndx, bool on )
{
    if( ndx < ARRAY_COUNT( s_gpio_cfg_led ) )
    {
        GPIO_OutPut( &s_gpio_cfg_led[ndx], on ? 0 : ~0 );
    }
}

float_t board_temp_sensor_resistance( float_t therm_time, float_t ref_time )
{
    const float_t comparison_resistance = 1000.0f;  // R1
    return comparison_resistance * therm_time / ref_time;
}

uint16_t board_crc( const void * p_data, uint32_t size )
{
    CRC16_Init( true, true );
    CRC16_Reseed( 0 );
    CRC16_AddDataArray( (uint32_t*)p_data, size >> 2 );
    return CRC16_GetCRC();
}

void board_reset( void )
{
    NVIC_SystemReset();
}


void uart_write_cb( int status )
{
    s_p_uart_write_cb();
}

void board_uart_write_lock( void )
{
    NVIC_DisableIRQ( PMU_IRQn );
}

void board_uart_write_unlock( void )
{
    NVIC_EnableIRQ( PMU_IRQn );
}

void board_uart_write( void * p_data, uint16_t size, uart_write_cb_t p_uart_write_cb )
{
    static uint32_t uart_write_descriptor[] =
    {
        PMU_TRANSFER( PMU_INTERRUPT, PMU_STOP, PMU_TX_READ_8_BIT, PMU_TX_READ_INC, PMU_TX_WRITE_8_BIT, PMU_TX_WRITE_NO_INC, 0, (uint32_t)UART_FIFO, 0, BOARD_UART_FIFO_PMU_FLAG, BOARD_UART_TX_FIFO_LVL ),
    };
    static pmu_transfer_des_t * const p_des = (pmu_transfer_des_t*)uart_write_descriptor;
    p_des->read_address = (uint32_t)p_data;
    p_des->tx_length = (uint32_t)size;
    s_p_uart_write_cb = p_uart_write_cb;
    PMU_Start( BOARD_PMU_CHANNEL_UART_WRITE, uart_write_descriptor, uart_write_cb );

}

void board_final( void )
{
    NVIC_ClearPendingIRQ( GPIO_P0_IRQn );
    NVIC_EnableIRQ( GPIO_P0_IRQn );
}

void board_disable_sample_timer( void )
{
    TMR32_Stop( MXC_TMR1 );
}

void board_enable_sample_timer( void )
{
    TMR32_Start( MXC_TMR1 );
}
