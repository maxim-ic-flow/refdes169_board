#ifndef __LCD_H__
#define __LCD_H__


void lcd_init( void );
void lcd_off( void );
void lcd_on( void );
void lcd_write_cmd( uint8_t data );
void lcd_write_data( const uint8_t *p_data, uint8_t count );
void lcd_cursor( uint8_t col, uint8_t row );

#endif
