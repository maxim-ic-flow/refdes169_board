#ifndef _TDC_H_
#define _TDC_H_

#include "max3510x.h"

#pragma pack(1)

typedef struct _tof_result_t
{
    max3510x_measurement_t	up;         // 30 bytes
    max3510x_measurement_t	down;       // 30 bytes
    max3510x_fixed_t		tof_diff;   // 4 bytes

}
tof_result_t;

typedef struct _tdc_tof_result_t
{
    // evenly divisable by 4
    uint16_t                status;
    tof_result_t	        tof; 
    uint16_t                pad;
}
tdc_tof_result_t;

typedef struct _tdc_temperature_result_t
{
    // evenly divisable by 4
    uint16_t                status;
    max3510x_fixed_t	    temperature[2];
    uint16_t                pad;
}
tdc_temperature_result_t;

typedef struct _tdc_calibration_result_t
{
	uint16_t				status;
	max3510x_fixed_t	    calibration;
	uint16_t                pad;
}
tdc_calibration_result_t;

typedef union _tdc_result_t
{
    uint16_t                	status;
    tdc_temperature_result_t	temperature;
    tdc_tof_result_t 		    tof;
	tdc_calibration_result_t	calibration;
}
tdc_result_t;


#pragma pack()

void tdc_interrupt( void* );

void tdc_init( void );
void tdc_get_tof_result( tdc_tof_result_t * p_result );
void tdc_get_temperature_result( tdc_temperature_result_t * p_result );
void tdc_get_calibration_result( tdc_calibration_result_t *p_result );
void tdc_configure( const max3510x_registers_t * p_config );

void tdc_set_sfreq( uint16_t sfreq );
uint16_t tdc_get_sfreq( void );
void tdc_set_dreq( uint16_t v );
uint16_t tdc_get_dreq( void );
void tdc_set_hreg_d( uint16_t v );
uint16_t tdc_get_hreg_d( void );
void tdc_set_vs( uint16_t v );
uint16_t tdc_get_vs( void );
void tdc_set_lt_n( uint16_t v );
uint16_t tdc_get_lt_n( void );
void tdc_set_lt_s( uint16_t v );
uint16_t tdc_get_lt_s( void );
void tdc_set_st( uint16_t v );
uint16_t tdc_get_st( void );
void tdc_set_50d( uint16_t v );
uint16_t tdc_get_50d( void );
void tdc_set_pecho( uint16_t v );
uint16_t tdc_get_pecho( void );
void tdc_set_afe_bp( uint16_t v );
uint16_t tdc_get_afe_bp( void );
void tdc_set_sd( uint16_t v );
uint16_t tdc_get_sd( void );
void tdc_set_afeout( uint16_t v );
uint16_t tdc_get_afeout( void );
void tdc_set_4m_bp( uint16_t v );
uint16_t tdc_get_4m_bp( void );
void tdc_set_f0( uint16_t v );
uint16_t tdc_get_f0( void );
void tdc_set_pga( uint16_t v );
uint16_t tdc_get_pga( void );
void tdc_set_lowq( uint16_t v );
uint16_t tdc_get_lowq( void );
void tdc_set_bp_bp( uint16_t v );
uint16_t tdc_get_bp_bp( void );
void tdc_set_pl( uint16_t v );
uint16_t tdc_get_pl( void );
void tdc_set_dpl( uint16_t v );
uint16_t tdc_get_dpl( void );
void tdc_set_stop_pol( uint16_t v );
uint16_t tdc_get_stop_pol( void );
void tdc_set_stop( uint16_t v );
uint16_t tdc_get_stop( void );
void tdc_set_t2wv( uint16_t v );
uint16_t tdc_get_t2wv( void );
void tdc_set_tof_cyc( uint16_t v );
uint16_t tdc_get_tof_cyc( void );
void tdc_set_timout( uint16_t v );
uint16_t tdc_get_timeout( void );
void tdc_set_hitwv( const uint8_t * p_hitwave );
void tdc_get_hitwv( uint8_t * p_hitwave );
void tdc_set_c_offsetupr( uint8_t v );
uint8_t tdc_get_c_offsetupr( void );
void tdc_set_c_offsetup( uint8_t v );
uint8_t tdc_get_c_offsetup( void );
void tdc_set_c_offsetdnr( uint8_t v );
uint8_t tdc_get_c_offsetdnr( void );
void tdc_set_c_offsetdn( uint8_t v );
uint8_t tdc_get_c_offsetdn( void );
void tdc_set_tdf( uint16_t v );
uint16_t tdc_get_tdf( void );
void tdc_set_tdm( uint16_t v );
uint16_t tdc_get_tdm( void );
void tdc_set_tmf( uint16_t v );
uint16_t tdc_get_tmf( void );
void tdc_set_tmm( uint16_t v );
uint16_t tdc_get_tmm( void );
void tdc_set_cal_use( uint16_t v );
uint16_t tdc_get_cal_use( void );
void tdc_set_cal_cfg( uint16_t v );
uint16_t tdc_get_cal_cfg( void );
void tdc_set_precyc( uint16_t v );
uint16_t tdc_get_precyc( void );
void tdc_set_portcyc( uint16_t v );
uint16_t tdc_get_portcyc( void );
void tdc_set_dly( uint16_t v );
uint16_t tdc_get_dly( void );
void tdc_set_cmp_en( uint16_t v );
uint16_t tdc_get_cmp_en( void );
void tdc_set_cmp_sel( uint16_t v );
uint16_t tdc_get_cmp_sel( void );
void tdc_set_et_cont( uint16_t v );
uint16_t tdc_get_et_cont( void );
void tdc_set_cont_int( uint16_t v );
uint16_t tdc_get_cont_int( void );
void tdc_set_clk_s( uint16_t v );
uint16_t tdc_get_clk_s( void );
void tdc_set_cal_period( uint16_t v );
uint16_t tdc_get_cal_period( void );
void tdc_set_32k_bp( uint16_t v );
uint16_t tdc_get_32k_bp( void );
void tdc_set_32k_en( uint16_t v );
uint16_t tdc_get_32k_en( void );
void tdc_set_eosc( uint16_t v );
uint16_t tdc_get_eosc( void );
void tdc_set_am( uint16_t v );
uint16_t tdc_get_am( void );
void tdc_set_wf( uint16_t v );
uint16_t tdc_get_wf( void );
void tdc_set_wd( uint16_t v );
uint16_t tdc_get_wd( void );
void tdc_cmd_bpcal( void );
void tdc_cmd_tof_diff( void );
void tdc_cmd_temperature( void );
void tdc_cmd_calibrate( void );
void tdc_cmd_tof_up( void );
void tdc_cmd_tof_down( void );
void tdc_start_event_engine( bool tof, bool temp );
void tdc_cmd_initialize( void );
void tdc_cmd_halt( void );
void tdc_cmd_read_config(  max3510x_registers_t * p_config );

void tdc_adjust_and_measure( uint8_t offset_up, uint8_t offset_down );
void tdc_read_thresholds( uint8_t *p_up, uint8_t *p_down );

typedef enum _tdc_last_cmd_t
{
    tdc_cmd_context_none,
    tdc_cmd_context_tof_diff,
	tdc_cmd_context_tof_up,
	tdc_cmd_context_tof_down,
    tdc_cmd_context_temperature,
	tdc_cmd_context_calibrate
}
tdc_cmd_context_t;

tdc_cmd_context_t tdc_cmd_context( void );


#endif
