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

void printLCD(hd44780_I2Cexp *lcd, String l1, String l2, String l3, String l4)
{
    if (!DISPLAY_MODE)
        return;

    lcd->setCursor(0, 0);
    while (l1.length() < LCD_COL)
        l1 = l1 + " ";
    while (l2.length() < LCD_COL)
        l2 = l2 + " ";
    while (l3.length() < LCD_COL)
        l3 = l3 + " ";
    while (l4.length() < LCD_COL)
        l4 = l4 + " ";
    lcd->print(l1);
    lcd->print(l2);
    lcd->print(l3);
    lcd->print(l4);
}

void lcdPrintWithNewlines(hd44780_I2Cexp *lcd, const char *text)
{
    if (!DISPLAY_MODE)
        return;

    int row = 0;
    int col = 0;
    char line[LCD_COL + 1] = {0}; // buffer for one line

    lcd->clear();

    for (int i = 0; text[i] != '\0' && row < LCD_ROW; i++)
    {
        if (text[i] == '\n' || col >= LCD_COL)
        {
            // Pad the rest of the line with spaces
            while (col < LCD_COL)
                line[col++] = ' ';
            line[LCD_COL] = '\0';

            lcd->setCursor(0, row);
            lcd->print(line);

            // Reset for next line
            row++;
            col = 0;
            memset(line, 0, sizeof(line));

            if (text[i] == '\n')
                continue;
        }

        line[col++] = text[i];
    }

    // Print last line if any remaining content
    if (row < LCD_ROW && col > 0)
    {
        while (col < LCD_COL)
            line[col++] = ' ';
        line[LCD_COL] = '\0';
        lcd->setCursor(0, row);
        lcd->print(line);
    }
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
