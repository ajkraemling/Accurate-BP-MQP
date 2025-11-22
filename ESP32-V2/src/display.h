#ifndef DISPLAY_H
#define DISPLAY_H

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "DisplayPresenter.h"
#include "config.h"

// Hardware adapter - implements IDisplay interface
class Display : public IDisplay
{
private:
    hd44780_I2Cexp lcd;

public:
    Display();
    bool begin();
    
    // IDisplay interface implementation
    void showLines(const char* line1, const char* line2 = nullptr,
                   const char* line3 = nullptr, const char* line4 = nullptr) override;
    void clear() override;
};

#endif