#ifndef __LCD_H__
#define __LCD_H__


uint8_t * lcd_aquire( void );
void lcd_release( const uint8_t *p_out );
void lcd_off( void );
void lcd_on( void );
void lcd_printf( const char * p_format, ... );

#endif
