#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "config.h"

class Display
{
private:
    hd44780_I2Cexp lcd;

public:
    Display();
    bool begin();
    void clear();
    void print(const String &line1, const String &line2 = "",
               const String &line3 = "", const String &line4 = "");
    void showCountdown(int seconds);
};

#endif