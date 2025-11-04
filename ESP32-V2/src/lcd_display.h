#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

bool initializeLCD(hd44780_I2Cexp *lcd);

void printLCD(hd44780_I2Cexp *lcd, String msg);

void printSignalWarnings(hd44780_I2Cexp *lcd, bool rangeOK, bool heartRateOK, bool stabilityOK);

#endif