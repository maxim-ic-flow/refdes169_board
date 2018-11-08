/*******************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include "global.h"
#include "lcd.h"
#include "board.h"

#include "pmu.h"
#include "spim.h"
#include "spidefs.h"

#include <stdarg.h>
#include <stdio.h>


#define LCD_BUFFER_SIZE (3*16)  // number of characters supported by the LCD.  must be evenly divisible by LCD_SPI_PAGE_SIZE
#define LCD_SPI_PAGE_SIZE 16    // must corrospond to a supported SPI controller page size.  see MXC_S_SPIM_MSTR_CFG_PAGE_* defines in spim_regs.h
#define LCD_SPI_FIFO_LEVEL  MXC_CFG_SPIM_FIFO_DEPTH/2   // used to coordinate the SPI FIFO config with the PMU config.

static uint8_t s_out[LCD_BUFFER_SIZE];

uint8_t * lcd_aquire( void )
{

    // this version of the LCD module has a single static buffer.
    return s_out;
}

void lcd_release( const uint8_t *p_out )
{
    // Releases an LCD buffer and sends the contents to the LCD.
    // This code sends an entire display's worth of data at once (all three lines).
    // This method provides the best efficiency in terms of MCU
    // processing overhead.
    //
    // This version of the LCD module has a single static buffer.
    //
    // Note this routine is not synchronized with hardware.
    // It takes about 800us for the data to be transfered to the LCD.
    // Subsequent calls to functions that manipulate s_out or the PMU channel hardware
    // before the current process is complete will cause LCD display errors.

    static const uint16_t lcd_write_spi_header = SPI_HEADER( SPI_DIR_TX, SPI_PAGES, LCD_BUFFER_SIZE / LCD_SPI_PAGE_SIZE, SPI_SS_DISASSERT );
    static const uint32_t lcd_write_pmu_descriptor[] =
    {
        // PMU descriptor that sends text data in s_out to the LCD
        // send the SPI header
        PMU_TRANSFER( PMU_NO_INTERRUPT, PMU_NO_STOP,
                      PMU_TX_READ_16_BIT, PMU_TX_READ_INC,
                      PMU_TX_WRITE_16_BIT, PMU_TX_WRITE_NO_INC,
                      sizeof(lcd_write_spi_header),
                      (uint32_t)BOARD_LCD_SPI_FIFO,
                      (uint32_t)&lcd_write_spi_header,
                      BOARD_LCD_SPI_TX_PMU_FLAG, sizeof(lcd_write_spi_header) ),
        // send LCD data
        PMU_TRANSFER( PMU_NO_INTERRUPT, PMU_STOP,
                      PMU_TX_READ_32_BIT, PMU_TX_READ_INC,
                      PMU_TX_WRITE_32_BIT, PMU_TX_WRITE_NO_INC,
                      sizeof(s_out),
                      (uint32_t)BOARD_LCD_SPI_FIFO,
                      (uint32_t)s_out,
                      BOARD_LCD_SPI_TX_PMU_FLAG, LCD_SPI_FIFO_LEVEL )
    };
    // Tell the PMU to exeucte the above descriptor
    PMU_Start( BOARD_PMU_CHANNEL_LCD, lcd_write_pmu_descriptor, NULL );
}

void lcd_off( void )
{
    board_lcd_power( false );
}

static void write_byte( uint8_t cmd )
{
    static spim_req_t s_req =
    {
        .deass = 1,
        .width = SPIM_WIDTH_1,
        .len = 1,
    };
    uint8_t i;
    s_req.tx_data = &cmd;
    s_req.len = 1;
    if( (SPIM_Trans( BOARD_LCD_SPI, &s_req )) != s_req.len )
    {
        while( 1 ); // fatal error -- step into CSL to determine reason
    }
}

void lcd_on( void )
{
    static const uint8_t init[] = { 0x39, 0x1D, 0x7B, 0x55, 0x6C, 0x0C, 0x01, 0x06 };
    static const uint8_t delay[] = { 1, 1, 1, 1, 200, 1, 2, 1 };

    static const sys_cfg_t sys_cfg =
    {
        .clk_scale = CLKMAN_SCALE_AUTO,
        .io_cfg = IOMAN_SPIM2( 1, 1, 0, 0, 0, 0, 0, 0 )
    };
    static const spim_cfg_t spim_cfg_lcd = { 3, SPIM_SSEL0_LOW, 750000 };

    board_lcd_power( true );

    if( SPIM_Init( BOARD_LCD_SPI, &spim_cfg_lcd, &sys_cfg ) != E_NO_ERROR )
    {
        while( 1 ); // initialization failed -- step into CSL to determine the reason
    }
    // LCD SPI page size is 16 bytes (1 line).
    // configure SPI FIFO controller to request fills when half-empty.
    // configure SCLK low time to be 2 clock periods.  This, in combination with the SCLK bitrate set above
    // allows enough time for the LCD controller to consume data without overflowing its receive buffer.
    // The LCD can accept a much higher SCLK, but it can not maintain maximum data rate for more than one byte.
    // 750kHz was experiemntally found to be the maximum maintainable bitrate.
    BOARD_LCD_SPI->mstr_cfg = (BOARD_LCD_SPI->mstr_cfg & ~(MXC_F_SPIM_MSTR_CFG_PAGE_SIZE | MXC_F_SPIM_MSTR_CFG_SCK_LO_CLK) |
                               MXC_S_SPIM_MSTR_CFG_PAGE_16B | (2 << MXC_F_SPIM_MSTR_CFG_SCK_LO_CLK_POS));
    BOARD_LCD_SPI->fifo_ctrl = (BOARD_LCD_SPI->fifo_ctrl & ~(MXC_F_SPIM_FIFO_CTRL_TX_FIFO_AE_LVL)) |
                               (LCD_SPI_FIFO_LEVEL << MXC_F_SPIM_FIFO_CTRL_TX_FIFO_AE_LVL_POS);

    vTaskDelay( portDELAY_MS( 40 ) );

    board_lcd_rs_cmd(); // command mode
    for( uint8_t i = 0; i < ARRAY_COUNT( init ); i++ )
    {
        write_byte( init[i] );
        vTaskDelay( portDELAY_MS( delay[i] ) );
    }
    board_lcd_rs_data(); // data mode
}

void lcd_printf( const char * p_format, ... )
{
    va_list args;
    va_start( args, p_format );
    uint8_t *p_buf = lcd_aquire();
    vsnprintf( p_buf, sizeof(s_out), p_format, args );
    lcd_release(p_buf);
    va_end( args );
}


