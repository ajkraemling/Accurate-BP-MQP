#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

bool initializeLCD(hd44780_I2Cexp *lcd);

void printLCD(hd44780_I2Cexp *lcd,
              String l1 = "                    ",
              String l2 = "                    ",
              String l3 = "                    ",
              String l4 = "                    ");

void lcdPrintWithNewlines(hd44780_I2Cexp *lcd, const char *text);

void printSignalWarnings(hd44780_I2Cexp *lcd, bool rangeOK, bool heartRateOK, bool stabilityOK);

#endif