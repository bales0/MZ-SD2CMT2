#include "lcd.h"

#include <Arduino.h>
#include <LiquidCrystal.h>

#define LCD_RS 8
#define LCD_EN 9

#define LCD_D4 4
#define LCD_D5 5
#define LCD_D6 6
#define LCD_D7 7

static LiquidCrystal lcd(
    LCD_RS,
    LCD_EN,
    LCD_D4,
    LCD_D5,
    LCD_D6,
    LCD_D7
);

void lcd_init(void)
{
    lcd.begin(16, 2);
    lcd.clear();
}

void lcd_clear(void)
{
    lcd.clear();
}

void lcd_home(void)
{
    lcd.home();
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    lcd.setCursor(col, row);
}

void lcd_print(const char *text)
{
    lcd.print(text);
}