#include "Display.h"

Display::Display() : lcd(LCD_I2C_ADDR) {}

bool Display::begin()
{
    int status = lcd.begin(LCD_COLS, LCD_ROWS);
    if (status)
    {
        return false;
    }
    return true;
}

void Display::clear()
{
    lcd.clear();
}

void Display::showLines(const char* line1, const char* line2,
                        const char* line3, const char* line4)
{
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print(line1);
    
    if (line2 != nullptr)
    {
        lcd.setCursor(0, 1);
        lcd.print(line2);
    }
    
    if (line3 != nullptr)
    {
        lcd.setCursor(0, 2);
        lcd.print(line3);
    }
    
    if (line4 != nullptr)
    {
        lcd.setCursor(0, 3);
        lcd.print(line4);
    }
}