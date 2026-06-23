#ifndef SD2CMT2_LCD_H
#define SD2CMT2_LCD_H

#include <stdint.h>

void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *text);

#endif