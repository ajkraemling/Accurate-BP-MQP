#include "Display.h"

Display::Display() : lcd(LCD_I2C_ADDR) {}

bool Display::begin()
{
    int status = lcd.begin(LCD_COLS, LCD_ROWS);
    if (status)
    {
        Serial.print("LCD init failed: ");
        Serial.println(status);
        return false;
    }
    return true;
}

void Display::clear()
{
    lcd.clear();
}

void Display::print(const String &line1, const String &line2,
                    const String &line3, const String &line4)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    if (line2.length() > 0)
    {
        lcd.setCursor(0, 1);
        lcd.print(line2);
    }
    if (line3.length() > 0)
    {
        lcd.setCursor(0, 2);
        lcd.print(line3);
    }
    if (line4.length() > 0)
    {
        lcd.setCursor(0, 3);
        lcd.print(line4);
    }
}

void Display::showCountdown(int seconds)
{
    print("CALIBRATION", "Place finger on", "sensor...",
          String(seconds) + "...");
}
