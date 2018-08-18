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
#ifndef __BOARD_H__

#include "max3510x.h"
#include "max32625.h"
#include "spim_regs.h"
#include "rtc.h"


#define BOARD_PMU_CHANNEL_TDC_SPI_WRITE	0
#define BOARD_PMU_CHANNEL_TDC_SPI_READ	1
#define BOARD_PMU_CHANNEL_UART_WRITE	2

#define BOARD_TDC_SPI	 				MXC_SPIM1
#define BOARD_TDC_SPI_FIFO   			MXC_SPIM1_FIFO
#define BOARD_SPI_FIFO_PMU_FLAG			PMU_WAIT_IRQ_MASK1_SEL0_SPI1_RX_FIFO_AF

#define BOARD_UART_FIFO_PMU_FLAG		PMU_WAIT_IRQ_MASK1_SEL0_UART1_TX_FIFO_AE
#define UART_NDX								1
#define BOARD_UART								MXC_UART1
#define UART_IRQ								UART1_IRQn
#define UART_FIFO								MXC_BASE_UART1_FIFO

#define BOARD_MAX3510X_CLOCK_FREQ               4000000 // nominal frequency of the max35104's high speed crystal

#define BOARD_UART_TX_FIFO_LVL		(MXC_UART_FIFO_DEPTH>>1)
#define BOARD_SPI_RX_FIFO_LVL		(MXC_CFG_SPIM_FIFO_DEPTH>>1)



#define BOARD_EVENT_SYSTICK		(1<<0)
#define BOARD_EVENT_MAX35104 	(1<<1)
#define BOARD_EVENT_BUTTON		(1<<2)
#define BOARD_EVENT_UART		(1<<3)

#ifdef ENABLE_LP1_IDLE
__STATIC_INLINE uint32_t board_timestamp(void)
{
	return RTC_GetCount();
}
#else
uint32_t board_timestamp(void);
#endif

void board_enable_sample_timer(void);
void board_disable_sample_timer(void);

void board_tdc_disable_interrupt(void);
void board_tdc_enable_interrupt(void);

typedef void (*uart_write_cb_t)(void);
void board_uart_write( void *p_data, uint16_t size, uart_write_cb_t p_uart_write_cb );
void board_uart_write_lock( void );
void board_uart_write_unlock( void );

void board_uart_enable_interrupt(void);
void board_uart_disable_interrupt(void);

void board_led( uint8_t ndx, bool on );


void board_init( void );
void board_final( void );
uint32_t board_sleep( void );
	


void board_wait_ms( uint32_t ms );

void board_printf( const char *p_format, ... );

uint16_t board_uart_read( void *pv, uint16_t length );


float_t board_elapsed_time( uint32_t timestamp, float_t *p_elapsed );
bool board_flash_write( const void *p_data, uint16_t size );
void board_flash_read( void *p_data, uint16_t size );
uint16_t board_crc( const void * p_data, uint32_t size );

uint16_t board_max3510x_interrupt_status( void );
float_t board_temp_sensor_resistance( float_t therm_time, float_t ref_time );
float_t board_clock_set( float_t frequency );
void board_clock_enable(bool);
bool board_switch( uint8_t switch_ndx, bool * p_changed );

void board_reset(void);

#endif

