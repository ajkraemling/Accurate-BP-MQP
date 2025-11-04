#include "lcd_display.h"
#include "config.h"

bool initializeLCD(hd44780_I2Cexp *lcd)
{
    int status = lcd->begin(LCD_COL, LCD_ROW);
    if (status)
    {
        Serial.print("LCD initialization failed, status: ");
        Serial.println(status);
    }
    return status;
}

void printLCD(hd44780_I2Cexp *lcd, String msg)
{
    lcd->setCursor(0, 1);
    lcd->print(msg);
}

void printSignalWarnings(hd44780_I2Cexp *lcd, bool rangeOK, bool heartRateOK, bool stabilityOK)
{
    lcd->clear(); // Clear previous messages
    delay(10);
    lcd->setCursor(0, 0); // Row 0
    lcd->print("Signal Status:");

    if (!(rangeOK && heartRateOK && stabilityOK))
    {
        lcd->setCursor(0, 1);
        lcd->print("!!! WARNING !!!");

        // Row 2
        lcd->setCursor(0, 2);
        if (!rangeOK)
            lcd->print("Range issue       "); // pad spaces to overwrite old text
        else
            lcd->print("                    "); // clear old message

        // Row 3
        lcd->setCursor(0, 3);
        if (!heartRateOK && !stabilityOK)
        {
            lcd->print("HR & Stability    ");
        }
        else if (!heartRateOK)
        {
            lcd->print("Heart rate issue  ");
        }
        else if (!stabilityOK)
        {
            lcd->print("Signal unstable   ");
        }
        else
        {
            lcd->print("                    "); // nothing to show
        }
    }
    else
    {
        lcd->setCursor(0, 1);
        lcd->print("Signal OK         ");
        lcd->setCursor(0, 2);
        lcd->print("                    ");
        lcd->setCursor(0, 3);
        lcd->print("                    ");
    }
}
