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
#include "board.h"
#include "config.h"
#include "flow.h"

#include "pmu.h"
#include "spim.h"

#include "tdc.h"
#include "max3510x.h"
#include "spidefs.h"

static tdc_cmd_context_t s_last_cmd;

static tdc_tof_result_t             s_tof_result;
static tdc_temperature_result_t     s_temperature_result;
static tdc_calibration_result_t     s_calibration_result;

static SemaphoreHandle_t        s_bus_lock;
static SemaphoreHandle_t        s_cmd_lock;

static const uint32_t s_tof_diff_descriptor[] =
{
    // PMU descriptor to read out the SPI data from the TDC's TOF registers
    PMU_TRANSFER( PMU_INTERRUPT, PMU_STOP,
                  PMU_TX_READ_16_BIT, PMU_TX_READ_NO_INC,
                  PMU_TX_WRITE_32_BIT, PMU_TX_WRITE_INC,
                  sizeof(s_tof_result.status) + sizeof(s_tof_result.tof),
                  (uint32_t)&s_tof_result,
                  (uint32_t)&BOARD_TDC_SPI_FIFO->rslts_32,
                  BOARD_TDC_SPI_RX_FIFO_PMU_FLAG, 2 )
};

static const uint32_t s_temperature_descriptor[] =
{
    // PMU descriptor to read out the SPI data from the TDC's temperature registers
    PMU_TRANSFER( PMU_INTERRUPT, PMU_STOP,
                  PMU_TX_READ_16_BIT, PMU_TX_READ_NO_INC,
                  PMU_TX_WRITE_32_BIT, PMU_TX_WRITE_INC,
                  sizeof(s_temperature_result.status) + sizeof(s_temperature_result.temperature),
                  (uint32_t)&s_temperature_result,
                  (uint32_t)&BOARD_TDC_SPI_FIFO->rslts_32,
                  BOARD_TDC_SPI_RX_FIFO_PMU_FLAG, 2 )
};

static const uint32_t s_calibration_descriptor[] =
{
    // PMU descriptor to read out the SPI data from the TDC's calibration registers
    PMU_TRANSFER( PMU_INTERRUPT, PMU_STOP,
                  PMU_TX_READ_16_BIT, PMU_TX_READ_NO_INC,
                  PMU_TX_WRITE_32_BIT, PMU_TX_WRITE_INC,
                  sizeof(s_calibration_result.status) + sizeof(s_calibration_result.calibration),
                  (uint32_t)&s_calibration_result,
                  (uint32_t)&BOARD_TDC_SPI_FIFO->rslts_32,
                  BOARD_TDC_SPI_RX_FIFO_PMU_FLAG, 2 )
};

static const uint16_t s_read_tof_diff_results[] =
{
    // SPI meta and data to read out the TDC's TOF registers
    SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                           // send read command
    SPI_END | MAX3510X_OPCODE_READ_REG( MAX3510X_REG_INTERRUPT_STATUS ),          // status register address
    SPI_HEADER( SPI_DIR_RX, SPI_BYTES, sizeof(s_tof_result.status), SPI_SS_DISASSERT ),
    // read relevant TOF egisters
    SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                           // send read command
    SPI_END | MAX3510X_OPCODE_READ_REG( MAX3510X_REG_WVRUP ),                     // address of register array of interest
    SPI_HEADER( SPI_DIR_RX, SPI_PAGES, sizeof(s_tof_result.tof) >> 2, SPI_SS_DISASSERT ),  // read in register values
};

static const uint16_t s_read_temperature_results[] =
{
    // SPI meta and data to read out the TDC's temperature registers
    SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                           // send read command
    SPI_END | MAX3510X_OPCODE_READ_REG( MAX3510X_REG_INTERRUPT_STATUS ),          // status register address
    SPI_HEADER( SPI_DIR_RX, SPI_BYTES, sizeof(s_temperature_result.status), SPI_SS_DISASSERT ),

    SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                           // send read command
    SPI_END | MAX3510X_OPCODE_READ_REG( MAX3510X_REG_T1INT ),                     // T1 register
    SPI_HEADER( SPI_DIR_RX, SPI_BYTES, sizeof(s_temperature_result.temperature[0]), SPI_SS_DISASSERT ),  // read in register values
                                                                                                         // read T2 temperature registers
    SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                           // send read command
    SPI_END | MAX3510X_OPCODE_READ_REG( MAX3510X_REG_T2INT ),                     // T2 register
    SPI_HEADER( SPI_DIR_RX, SPI_BYTES, sizeof(s_temperature_result.temperature[1]), SPI_SS_DISASSERT ),  // read in register values
};


static const uint16_t s_read_calibration_results[] =
{
    // SPI meta and data to read out the TDC's calibration registers
    SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                              // send read command
    SPI_END | MAX3510X_OPCODE_READ_REG( MAX3510X_REG_INTERRUPT_STATUS ),                // status register address
    SPI_HEADER( SPI_DIR_RX, SPI_BYTES, 2, SPI_SS_DISASSERT ),                           // read status value

    SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                              // send read command
    SPI_END | MAX3510X_OPCODE_READ_REG( MAX3510X_REG_CALIBRATIONINT ),                  // calibration register
    SPI_HEADER( SPI_DIR_RX, SPI_BYTES, sizeof(s_calibration_result.calibration), SPI_SS_DISASSERT ), // read in register values
};
static const uint32_t s_toff_diff_write_descriptor[] =
{
    // PMU descriptor that sends SPI commands read out the TDC's TOF registers
    PMU_TRANSFER( PMU_NO_INTERRUPT, PMU_STOP,
                  PMU_TX_READ_16_BIT, PMU_TX_READ_INC,
                  PMU_TX_WRITE_16_BIT, PMU_TX_WRITE_NO_INC,
                  sizeof(s_read_tof_diff_results),
                  (uint32_t)BOARD_TDC_SPI_FIFO,
                  (uint32_t)s_read_tof_diff_results,
                  BOARD_TDC_SPI_TX_FIFO_PMU_FLAG, sizeof(s_read_tof_diff_results) )
};

static const uint32_t s_temperature_write_descriptor[] =
{
    // PMU descriptor that sends SPI commands read out the TDC's temperature registers
    PMU_TRANSFER( PMU_NO_INTERRUPT, PMU_STOP,
                  PMU_TX_READ_16_BIT, PMU_TX_READ_INC,
                  PMU_TX_WRITE_16_BIT, PMU_TX_WRITE_NO_INC,
                  sizeof(s_read_temperature_results),
                  (uint32_t)BOARD_TDC_SPI_FIFO,
                  (uint32_t)s_read_temperature_results,
                  BOARD_TDC_SPI_TX_FIFO_PMU_FLAG, sizeof(s_read_temperature_results) )
};

static const uint32_t s_calibration_write_descriptor[] =
{
    // PMU descriptor that sends SPI commands read out the TDC's temperature registers
    PMU_TRANSFER( PMU_NO_INTERRUPT, PMU_STOP,
                  PMU_TX_READ_16_BIT, PMU_TX_READ_INC,
                  PMU_TX_WRITE_16_BIT, PMU_TX_WRITE_NO_INC,
                  sizeof(s_read_calibration_results),
                  (uint32_t)BOARD_TDC_SPI_FIFO,
                  (uint32_t)s_read_calibration_results,
                  BOARD_TDC_SPI_TX_FIFO_PMU_FLAG, sizeof(s_read_calibration_results) )
};


static void unlock()
{
    xSemaphoreGive( s_bus_lock );
}


static void lock()
{
    xSemaphoreTake( s_bus_lock, portMAX_DELAY );
}

static void cmd_lock()
{
    xSemaphoreTake( s_cmd_lock, portMAX_DELAY );
}



void tdc_init( void )
{
    s_bus_lock = xSemaphoreCreateBinary();
    configASSERT( s_bus_lock );
    s_cmd_lock = xSemaphoreCreateBinary();
    configASSERT( s_cmd_lock );
    max3510x_reset( NULL );
    xSemaphoreGive( s_bus_lock );
    xSemaphoreGive( s_cmd_lock );
    max3510x_wait_for_reset_complete( NULL );
}

void tdc_configure( const max3510x_registers_t * p_config )
{
    lock();
    max3510x_write_config( NULL, p_config );
    unlock();
    tdc_cmd_bpcal();
}

static void pmu_write_complete_cb( int status )
{
    // ISR context
    // called when the PMU has completed writing to the TDC
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR( s_bus_lock, &woken );
    portYIELD_FROM_ISR( woken );
}

static void pmu_read_complete_cb( int status )
{
    // ISR context
    // called when the PMU has completed reading result data from the TDC
    BaseType_t woken = pdFALSE;
    flow_sample_complete(s_last_cmd);
    s_last_cmd = tdc_cmd_context_none;
    xSemaphoreGiveFromISR( s_cmd_lock, &woken );
    portYIELD_FROM_ISR( woken );
}

void tdc_interrupt( void * pv )
{
    BaseType_t woken = pdFALSE;
    switch (s_last_cmd)
    {
        case tdc_cmd_context_tof_up:
        case tdc_cmd_context_tof_down:
        case tdc_cmd_context_tof_diff:
        {
            PMU_Start( BOARD_PMU_CHANNEL_TDC_SPI_READ, s_tof_diff_descriptor, pmu_read_complete_cb );
            PMU_Start( BOARD_PMU_CHANNEL_TDC_SPI_WRITE, s_toff_diff_write_descriptor, NULL );
            break;
        }
        case tdc_cmd_context_temperature:
        {
            PMU_Start( BOARD_PMU_CHANNEL_TDC_SPI_READ, s_temperature_descriptor, pmu_read_complete_cb );
            PMU_Start( BOARD_PMU_CHANNEL_TDC_SPI_WRITE, s_temperature_write_descriptor, NULL );
            break;
        }
        case tdc_cmd_context_calibrate:
        {
            PMU_Start( BOARD_PMU_CHANNEL_TDC_SPI_READ, s_calibration_descriptor, pmu_read_complete_cb );
            PMU_Start( BOARD_PMU_CHANNEL_TDC_SPI_WRITE, s_calibration_write_descriptor, NULL );
            break;
        }
    }
    portYIELD_FROM_ISR( woken );
}

void tdc_read_thresholds( uint8_t * p_up, uint8_t * p_down )
{
    lock();
    max3510x_read_thresholds( NULL, p_up, p_down );
    unlock();
}


void tdc_adjust_and_measure( uint8_t offset_up, uint8_t offset_down )
{
    // adjusts early-edge offsets and sends a TOF_DIFF command
    //
    // return offsets are implicity set to zero

    static uint16_t write_offsets[] =
    {
        // SPI meta and data to set early edge thresholds and start a TOF_DIFF
        SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT ),                              // send write command
        SPI_END | MAX3510X_OPCODE_WRITE_REG( MAX3510X_REG_TOF6 ),                           // comparator offset register address
        SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 2 * sizeof(uint16_t), SPI_SS_DISASSERT ),          // send 4 bytes worth of offset info
        0,                                                                                  // up early offset/return
        0,                                                                                  // down early offset/resturn
        SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_DISASSERT ),                           // send TOF_DIF command
        SPI_END | MAX3510X_OPCODE_TOF_DIFF
    };
    static uint16_t * const p_offset = &write_offsets[3];
    static const uint32_t measure_descriptor[] =
    {
        // PMU descriptor that sends SPI commands read out the TDC's TOF registers
        PMU_TRANSFER( PMU_INTERRUPT, PMU_STOP,
                      PMU_TX_READ_16_BIT, PMU_TX_READ_INC,
                      PMU_TX_WRITE_16_BIT, PMU_TX_WRITE_NO_INC,
                      sizeof(write_offsets),
                      (uint32_t)BOARD_TDC_SPI_FIFO,
                      (uint32_t)write_offsets,
                      BOARD_TDC_SPI_TX_FIFO_PMU_FLAG, 2 )
    };
    p_offset[0] = MAX3510X_ENDIAN( MAX3510X_REG_SET( TOF6_C_OFFSETUP, offset_up ) );
    p_offset[1] = MAX3510X_ENDIAN( MAX3510X_REG_SET( TOF7_C_OFFSETDN, offset_down ) );
    cmd_lock();
    lock();
    s_last_cmd = tdc_cmd_context_tof_diff;
    PMU_Start( BOARD_PMU_CHANNEL_TDC_SPI_WRITE, measure_descriptor, pmu_write_complete_cb );
}

void tdc_get_tof_result( tdc_tof_result_t * p_result )
{
    uint32_t * p_src = (uint32_t*)&s_tof_result;
    uint32_t * p_dst = (uint32_t*)p_result;
    // adjust for endian difference between the TDC and the micro
#if !defined(__BIG_ENDIAN)
    for( uint32_t i = 0; i < sizeof(s_tof_result) / sizeof(uint32_t); i++ ) p_dst[i] = __REV16( p_src[i] );
#endif
}

void tdc_get_calibration_result( tdc_calibration_result_t * p_result )
{
    uint32_t * p_src = (uint32_t*)&s_calibration_result;
    uint32_t * p_dst = (uint32_t*)p_result;
    // adjust for endian difference between the TDC and the micro
#if !defined(__BIG_ENDIAN)
    for( uint32_t i = 0; i < sizeof(s_tof_result) / sizeof(uint32_t); i++ ) p_dst[i] = __REV16( p_src[i] );
#endif
}

void tdc_get_temperature_result( tdc_temperature_result_t * p_result )
{
    uint32_t * p_src = (uint32_t*)&s_temperature_result;
    uint32_t * p_dst = (uint32_t*)p_result;
    // adjust for endian difference between the TDC and the micro
#if !defined(__BIG_ENDIAN)
    for( uint32_t i = 0; i < sizeof(s_temperature_result) / sizeof(uint32_t); i++ ) p_dst[i] = __REV16( p_src[i] );
#endif
}

void tdc_set_sfreq( uint16_t sfreq )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER1, SFREQ, sfreq );
    unlock();
}

uint16_t tdc_get_sfreq( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER1, SFREQ );
    unlock();
    return r;
}

void tdc_set_dreq( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER1, DREQ, v );
    unlock();
}

uint16_t tdc_get_dreq( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER1, DREQ );
    unlock();
    return r;
}

void tdc_set_hreg_d( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER1, HREG_D, v );
    unlock();
}

uint16_t tdc_get_hreg_d( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER1, HREG_D );
    unlock();
    return r;
}

void tdc_set_vs( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER1, VS, v );
    unlock();
}

uint16_t tdc_get_vs( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER1, VS );
    unlock();
    return r;
}

void tdc_set_lt_n( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER2, LT_N, v );
    unlock();
}

uint16_t tdc_get_lt_n( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER2, LT_N );
    unlock();
    return r;
}

void tdc_set_lt_s( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER2, LT_S, v );
    unlock();
}

uint16_t tdc_get_lt_s( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER2, LT_S );
    unlock();
    return r;
}
void tdc_set_st( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER2, ST, v );
    unlock();
}

uint16_t tdc_get_st( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER2, ST );
    unlock();
    return r;
}
void tdc_set_50d( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER2, LT_50D, v );
    unlock();
}

uint16_t tdc_get_50d( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER2, LT_50D );
    unlock();
    return r;
}
void tdc_set_pecho( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, SWITCHER2, PECHO, v );
    unlock();
}

uint16_t tdc_get_pecho( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, SWITCHER2, PECHO );
    unlock();
    return r;
}
void tdc_set_afe_bp( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE1, AFE_BP, v );
    unlock();
}

uint16_t tdc_get_afe_bp( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE1, AFE_BP );
    unlock();
    return r;
}
void tdc_set_sd( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE1, SD_EN, v );
    unlock();
}

uint16_t tdc_get_sd( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE1, SD_EN );
    unlock();
    return r;
}
void tdc_set_afeout( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE1, AFEOUT, v );
    unlock();
}

uint16_t tdc_get_afeout( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE1, AFEOUT );
    unlock();
    return r;
}
void tdc_set_4m_bp( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE2, 4M_BP, v );
    unlock();
}

uint16_t tdc_get_4m_bp( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE2, 4M_BP);
    unlock();
    return r;
}
void tdc_set_f0( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE2, F0, v );
    unlock();
}

uint16_t tdc_get_f0( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE2, F0 );
    unlock();
    return r;
}
void tdc_set_pga( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE2, PGA, v );
    unlock();
}

uint16_t tdc_get_pga( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE2, PGA );
    unlock();
    return r;
}
void tdc_set_lowq( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE2, LOWQ, v );
    unlock();
}

uint16_t tdc_get_lowq( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE2, LOWQ );
    unlock();
    return r;
}
void tdc_set_bp_bp( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, AFE2, BP_BYPASS, v );
    unlock();
}

uint16_t tdc_get_bp_bp( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, AFE2, BP_BYPASS );
    unlock();
    return r;
}
void tdc_set_pl( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF1, PL, v );
    unlock();
}

uint16_t tdc_get_pl( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, TOF1, PL );
    unlock();
    return r;
}
void tdc_set_dpl( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF1, DPL, v );
    unlock();
}

uint16_t tdc_get_dpl( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, TOF1, DPL );
    unlock();
    return r;
}
void tdc_set_stop_pol( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF1, STOP_POL, v );
    unlock();
}

uint16_t tdc_get_stop_pol( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, TOF1, STOP_POL );
    unlock();
    return r;
}
void tdc_set_stop( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF2, STOP, v );
    unlock();
}

uint16_t tdc_get_stop( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, TOF2, STOP );
    unlock();
    return r;
}
void tdc_set_t2wv( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF2, T2WV, v );
    unlock();
}

uint16_t tdc_get_t2wv( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, TOF2, T2WV );
    unlock();
    return r;
}
void tdc_set_tof_cyc( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF2, TOF_CYC, v );
    unlock();
}

uint16_t tdc_get_tof_cyc( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, TOF2, TOF_CYC );
    unlock();
    return r;
}
void tdc_set_timout( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF2, TIMOUT, v );
    unlock();
}

uint16_t tdc_get_timeout( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, TOF2, TIMOUT );
    unlock();
    return r;
}

void tdc_set_hitwv( const uint8_t * p_hitwave )
{
    lock();
    max3510x_set_hitwaves( NULL, p_hitwave );
    unlock();
}

void tdc_get_hitwv( uint8_t * p_hitwave )
{
    lock();
    max3510x_get_hitwaves( NULL, p_hitwave );
    unlock();
}

void tdc_set_c_offsetupr( uint8_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF6, C_OFFSETUPR, v );
    unlock();
}

uint8_t tdc_get_c_offsetupr( void )
{
    lock();
    int8_t r = MAX3510X_READ_BITFIELD( NULL, TOF6, C_OFFSETUPR );
    unlock();
    return r;
}

void tdc_set_c_offsetup( uint8_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF6, C_OFFSETUP, v );
    unlock();
}

uint8_t tdc_get_c_offsetup( void )
{
    lock();
    int8_t r = MAX3510X_READ_BITFIELD( NULL, TOF6, C_OFFSETUP );
    unlock();
    return r;
}

void tdc_set_c_offsetdnr( uint8_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF7, C_OFFSETDNR, v );
    unlock();
}

uint8_t tdc_get_c_offsetdnr( void )
{
    lock();
    int8_t r = MAX3510X_READ_BITFIELD( NULL, TOF7, C_OFFSETDNR );
    unlock();
    return r;
}

void tdc_set_c_offsetdn( uint8_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF7, C_OFFSETDN, v );
    unlock();
}

uint8_t tdc_get_c_offsetdn( void )
{
    lock();
    int8_t r = MAX3510X_READ_BITFIELD( NULL, TOF7, C_OFFSETDN );
    unlock();
    return r;
}


void tdc_set_tdf( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_1, TDF, v );
    unlock();
}

uint16_t tdc_get_tdf( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_1, TDF );
    unlock();
    return r;
}

void tdc_set_tdm( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_1, TDM, v );
    unlock();
}

uint16_t tdc_get_tdm( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_1, TDM );
    unlock();
    return r;
}

void tdc_set_tmf( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_1, TMF, v );
    unlock();
}

uint16_t tdc_get_tmf( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_1, TMF );
    unlock();
    return r;
}

void tdc_set_tmm( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_2, TMM, v );
    unlock();
}

uint16_t tdc_get_tmm( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_2, TMM );
    unlock();
    return r;
}

void tdc_set_cal_use( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_2, CAL_USE, v );
    unlock();
}

uint16_t tdc_get_cal_use( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_2, CAL_USE );
    unlock();
    return r;
}

void tdc_set_cal_cfg( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_2, CAL_CFG, v );
    unlock();
}

uint16_t tdc_get_cal_cfg( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_2, CAL_CFG );
    unlock();
    return r;
}

void tdc_set_precyc( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_2, PRECYC, v );
    unlock();
}

uint16_t tdc_get_precyc( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_2, PRECYC );
    unlock();
    return r;
}

void tdc_set_portcyc( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, EVENT_TIMING_2, PORTCYC, v );
    unlock();
}

uint16_t tdc_get_portcyc( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, EVENT_TIMING_2, PORTCYC );
    unlock();
    return r;
}

void tdc_set_dly( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, TOF_MEASUREMENT_DELAY, DLY, v );
    unlock();
}

uint16_t tdc_get_dly( void )
{
    lock();
    int16_t r = MAX3510X_READ_BITFIELD( NULL, TOF_MEASUREMENT_DELAY, DLY );
    unlock();
    return r;
}

void tdc_set_cmp_en( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, CALIBRATION_CONTROL, CMP_EN, v );
    unlock();
}

uint16_t tdc_get_cmp_en( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, CALIBRATION_CONTROL, CMP_EN );
    unlock();
    return r;
}

void tdc_set_cmp_sel( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, CALIBRATION_CONTROL, CMP_SEL, v );
    unlock();
}

uint16_t tdc_get_cmp_sel( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, CALIBRATION_CONTROL, CMP_SEL );
    unlock();
    return r;
}

void tdc_set_et_cont( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, CALIBRATION_CONTROL, ET_CONT, v );
    unlock();
}

uint16_t tdc_get_et_cont( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, CALIBRATION_CONTROL, ET_CONT );
    unlock();
    return r;
}

void tdc_set_cont_int( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, CALIBRATION_CONTROL, CONT_INT, v );
    unlock();
}

uint16_t tdc_get_cont_int( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, CALIBRATION_CONTROL, CONT_INT );
    unlock();
    return r;
}

void tdc_set_clk_s( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, CALIBRATION_CONTROL, CLK_S, v );
    unlock();
}

uint16_t tdc_get_clk_s( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, CALIBRATION_CONTROL, CLK_S );
    unlock();
    return r;
}

void tdc_set_cal_period( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, CALIBRATION_CONTROL, CAL_PERIOD, v );
    unlock();
}

uint16_t tdc_get_cal_period( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, CALIBRATION_CONTROL, CAL_PERIOD );
    unlock();
    return r;
}

void tdc_set_32k_bp( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, RTC, 32K_BP, v );
    unlock();
}

uint16_t tdc_get_32k_bp( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, RTC, 32K_BP);
    unlock();
    return r;
}

void tdc_set_32k_en( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, RTC, 32K_EN, v );
    unlock();
}

uint16_t tdc_get_32k_en( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, RTC, 32K_EN);
    unlock();
    return r;
}

void tdc_set_eosc( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, RTC, EOSC, v );
    unlock();
}

uint16_t tdc_get_eosc( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, RTC, EOSC );
    unlock();
    return r;
}

void tdc_set_am( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, RTC, AM, v );
    unlock();
}

uint16_t tdc_get_am( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, RTC, AM );
    unlock();
    return r;
}

void tdc_set_wf( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, RTC, WF, v );
    unlock();
}

uint16_t tdc_get_wf( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, RTC, WF );
    unlock();
    return r;
}

void tdc_set_wd( uint16_t v )
{
    lock();
    MAX3510X_WRITE_BITFIELD( NULL, RTC, WD_EN, v );
    unlock();
}

uint16_t tdc_get_wd( void )
{
    lock();
    uint16_t r = MAX3510X_READ_BITFIELD( NULL, RTC, WD_EN );
    unlock();
    return r;
}

void tdc_cmd_bpcal( void )
{
    const TickType_t _3ms = 3 / portTICK_PERIOD_MS + 2;
    lock();
    max3510x_bandpass_calibrate( NULL );
    vTaskDelay( _3ms );
    unlock();
}

void tdc_start_event_engine( bool tof, bool temp )
{
    lock();
    if( tof && temp )
    {
        max3510x_event_timing( NULL, max3510x_event_timing_mode_tof_temp );
    }
    else if( tof )
    {
        max3510x_event_timing( NULL, max3510x_event_timing_mode_tof );

    }
    unlock();
}

void tdc_cmd_calibrate( void )
{
    cmd_lock();
    lock();
    s_last_cmd = tdc_cmd_context_calibrate;
    max3510x_calibrate( NULL );
    unlock();
}

void tdc_cmd_temperature( void )
{
    cmd_lock();
    lock();
    s_last_cmd = tdc_cmd_context_temperature;
    max3510x_temperature( NULL );
    unlock();
}

void tdc_cmd_tof_diff( void )
{
    cmd_lock();
    lock();
    s_last_cmd = tdc_cmd_context_tof_diff;
    max3510x_tof_diff( NULL );
    unlock();
}

void tdc_cmd_tof_up( void )
{
    cmd_lock();
    lock();
    s_last_cmd = tdc_cmd_context_tof_up;
    max3510x_tof_up( NULL );
    unlock();
}

void tdc_cmd_tof_down( void )
{
    cmd_lock();
    lock();
    s_last_cmd = tdc_cmd_context_tof_down;
    max3510x_tof_down( NULL );
    unlock();
}

void tdc_cmd_read_config(  max3510x_registers_t * p_config )
{
    lock();
    max3510x_read_config( NULL, p_config );
    unlock();
}

static bool tdc_spi_test( void )
{
    uint16_t write = ~0;
    uint16_t read;
    lock();
    uint16_t original = max3510x_read_register( NULL, MAX3510X_REG_TOF_MEASUREMENT_DELAY );
    while( write )
    {
        max3510x_write_register( NULL, MAX3510X_REG_TOF_MEASUREMENT_DELAY, write );
        read = max3510x_read_register( NULL, MAX3510X_REG_TOF_MEASUREMENT_DELAY );
        if(  read != write )
        {
            return true;
        }
        write--;
    }
    max3510x_write_register( NULL, MAX3510X_REG_TOF_MEASUREMENT_DELAY, original );
    unlock();
    return true;
}

void tdc_cmd_halt( void )
{
    lock();
    max3510x_halt( NULL );
    unlock();
}

void tdc_cmd_initialize( void )
{
    lock();
    max3510x_initialize( NULL );
    unlock();
}

