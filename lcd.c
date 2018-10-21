#include "global.h"
#include "lcd.h"
#include "board.h"

#include "tmr.h"
#include "pmu.h"
#include "spim.h"
#include "spidefs.h"


#pragma pack(1)


#pragma pack()

#define LCD_BUFFER_SIZE (3*16)


static uint8_t          s_buffer[LCD_BUFFER_SIZE] = { 'Y' };
static uint32_t         s_out[LCD_BUFFER_SIZE];

static const uint32_t s_pmu_descriptor[] = 
{
    // 0
    PMU_MOVE( PMU_NO_INTERRUPT, PMU_NO_STOP,
              PMU_MOVE_READ_32_BIT, PMU_MOVE_READ_INC,
              PMU_MOVE_WRITE_32_BIT, PMU_MOVE_WRITE_NO_INC,
              PMU_MOVE_NO_CONT, 
              sizeof(s_out[0]),
              BOARD_LCD_SPI_FIFO,
              (uint32_t)s_out
            ), // 3 dwords
    // 3
    PMU_WRITE( PMU_NO_INTERRUPT, PMU_NO_STOP,
               PMU_WRITE_OR_MASK,
               MXC_TMR_GET_BASE(BOARD_TIMER_NDX_LCD),
               MXC_F_TMR_CTRL_ENABLE0, // value
               MXC_F_TMR_CTRL_ENABLE0 // mask
             ), // 4 dwords
    // 7
    PMU_WAIT( PMU_NO_INTERRUPT, PMU_NO_STOP,
              PMU_WAIT_SEL_0, 
              0, BOARD_TIMER_LCD_PMU_FLAG,
              0 
            ), // 4 dwords
    // 11
    PMU_MOVE( PMU_NO_INTERRUPT, PMU_NO_STOP,
              PMU_MOVE_READ_32_BIT, PMU_MOVE_READ_INC,
              PMU_MOVE_WRITE_32_BIT, PMU_MOVE_WRITE_NO_INC,
              PMU_MOVE_CONT, 
              sizeof(s_out[0]), 
              BOARD_LCD_SPI_FIFO, 
              (uint32_t)s_out
            ), // 3 dwords
    // 14
    PMU_WRITE( PMU_NO_INTERRUPT, PMU_NO_STOP,
               PMU_WRITE_OR_MASK,
               MXC_TMR_GET_BASE(BOARD_TIMER_NDX_LCD)+MXC_R_TMR_OFFS_INTFL,
               MXC_F_TMR_INTFL_TIMER0, // value
               MXC_F_TMR_INTFL_TIMER0 // mask
             ), // 4 dwords
    // 18
    PMU_LOOP( PMU_NO_INTERRUPT, PMU_NO_STOP,
              PMU_LOOP_SEL_COUNTER0, (uint32_t)&s_pmu_descriptor[3] 
             ),

    PMU_JUMP( PMU_INTERRUPT, PMU_STOP, (uint32_t)s_pmu_descriptor ) // 2 dwords
    // 20
};

static spim_req_t s_req =
{
    .deass = 1,
    .width = SPIM_WIDTH_1,
    .len = 1,
};


static void update(void)
{
    uint8_t *p_outd = (uint8_t*)s_out + 2;
    //lcd_write_data("test",4);
//    for(uint8_t i=0;i<ARRAY_COUNT(s_out);i++)
//    {
//        p_outd[i*sizeof(s_out[0])] = s_buffer[i];
//    }
    PMU_Start( BOARD_PMU_CHANNEL_LCD, s_pmu_descriptor, NULL );
}

void lcd_init( void )
{
    uint8_t i;
    for(i=0;i<ARRAY_COUNT(s_out);i++)
    {
        s_out[i] = ((SPI_END | '0'+i ) << 16) | SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_ASSERT );
    }
    s_out[i] = ((SPI_END | '*') << 16) | SPI_HEADER( SPI_DIR_TX, SPI_BYTES, 1, SPI_SS_DISASSERT );
    PMU_SetCounter( BOARD_PMU_CHANNEL_LCD, 0, i-2 );

}

void lcd_off( void )
{
    board_lcd_power( false );
}

void lcd_on( void )
{
    static const uint8_t init[] = { 0x39, 0x1D, 0x7B, 0x55, 0x6C, 0x0C, 0x01, 0x06 };
    static const uint8_t delay[] = { 1, 1, 1, 1, 200, 1, 2, 1 };
    
    lcd_init();
    board_lcd_power( true );
    vTaskDelay( portDELAY_MS(40) );
    for(uint8_t i=0;i<ARRAY_COUNT(init);i++)
    {
         lcd_write_cmd( init[i] );
        vTaskDelay( portDELAY_MS(delay[i]) );
    }
    board_lcd_rs_data();
    update();
}


void lcd_cursor( uint8_t col, uint8_t row )
{
    lcd_write_cmd( 0x80 + 0x10 * row + col);
}

void lcd_write_data( const uint8_t *p_data, uint8_t count )
{
    board_lcd_rs_data();
    s_req.tx_data = p_data;
    s_req.len = count;
    if( (SPIM_Trans( BOARD_LCD_SPI, &s_req )) != s_req.len )
    {
        while( 1 ); // fatal error -- step into CSL to determine reason
    }
}

void lcd_write_cmd( uint8_t cmd )
{
    uint8_t i;
    board_lcd_rs_cmd();
    s_req.tx_data = &cmd;
    s_req.len = 1;
    if( (SPIM_Trans( BOARD_LCD_SPI, &s_req )) != s_req.len )
    {
        while( 1 ); // fatal error -- step into CSL to determine reason
    }
}


