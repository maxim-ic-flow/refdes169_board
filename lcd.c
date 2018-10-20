#include "global.h"
#include "lcd.h"
#include "board.h"

#include "spim.h"

static spim_req_t s_req =
{
    .deass = 1,
    .width = SPIM_WIDTH_1,
    .len = 1,
};

void lcd_off( void )
{
    board_lcd_power( false );
}

void lcd_on( void )
{
    static const uint8_t init[] = { 0x39, 0x1D, 0x7B, 0x55, 0x6C, 0x0C, 0x01, 0x06 };
    static const uint8_t delay[] = { 1, 1, 1, 1, 50, 1, 2, 1 };
    
    board_lcd_power( true );
    vTaskDelay( 1 + 40 / portTICK_PERIOD_MS );
    for(uint8_t i=0;i<ARRAY_COUNT(init);i++)
    {
         lcd_write_cmd( init[i] );
        vTaskDelay( 1 + delay[i] / portTICK_PERIOD_MS );
    }
    lcd_cursor( 0, 1 );
    lcd_write_data( "test", 4 );
}


void lcd_cursor( uint8_t col, uint8_t row )
{
    lcd_write_cmd( 0x80 + 0x10 * row + col);
}

void lcd_write_data( const uint8_t *p_data, uint8_t count )
{
    board_lcd_rs_data();
    for(uint8_t i=0;i<count;i++)
    {
        s_req.tx_data = &p_data[i];
        if( (SPIM_Trans( BOARD_LCD_SPI, &s_req )) != 1 )
        {
            while( 1 ); // fatal error -- step into CSL to determine reason
        }
    }
}

void lcd_write_cmd( uint8_t cmd )
{
    uint8_t i;
    board_lcd_rs_cmd();
    s_req.tx_data = &cmd;
    if( (SPIM_Trans( BOARD_LCD_SPI, &s_req )) != 1 )
    {
        while( 1 ); // fatal error -- step into CSL to determine reason
    }
}


