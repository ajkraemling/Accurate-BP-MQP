#pragma once
#include <TFT_eSPI.h>
#include <SPI.h>

class Display {
public:
    Display();

    bool begin();
    void drawMainPage();
    void updateValues(int systolic, int diastolic);

private:
    TFT_eSPI tft;

    const int SCREEN_WIDTH = 320;
    const int SCREEN_HEIGHT = 240;

    const uint16_t LIGHT_BLUE = 0x5D1F;

    const int boxWidth = 280;
    const int boxHeight = 70;
    const int boxSpacing = 15;

    const int topBoxX = 20;
    const int topBoxY = 70;
    const int bottomBoxY = 155;
};