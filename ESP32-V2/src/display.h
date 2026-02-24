#pragma once
#include <TFT_eSPI.h>
#include <SPI.h>
#include "DisplayPresenter.h"

class Display : public IDisplay {
public:
    Display();

    bool begin();
    void drawMainPage();
    void updateValues(int systolic, int diastolic);

    // IDisplay interface
    void showLines(const char* line1, const char* line2 = nullptr,
                   const char* line3 = nullptr, const char* line4 = nullptr) override;
    void clear() override;

private:
    TFT_eSPI tft;

    static constexpr int SCREEN_WIDTH  = 320;
    static constexpr int SCREEN_HEIGHT = 240;
    static constexpr uint16_t LIGHT_BLUE = 0x5D1F;
    static constexpr int boxWidth    = 280;
    static constexpr int boxHeight   = 70;
    static constexpr int boxSpacing  = 15;
    static constexpr int topBoxX     = 20;
    static constexpr int topBoxY     = 70;
    static constexpr int bottomBoxY  = 155;
};