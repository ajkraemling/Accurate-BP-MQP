#include "display.h"

Display::Display() : tft() {}

bool Display::begin() {
    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);   // Backlight ON

    tft.init();
    tft.setRotation(1);

    // Fix color orientation if needed
    tft.writecommand(0x36);
    tft.writedata(0xA0);

    drawMainPage();
    return true;
}

void Display::drawMainPage() {
    tft.fillScreen(LIGHT_BLUE);

    tft.setTextColor(TFT_BLACK, LIGHT_BLUE);
    tft.setFreeFont(&FreeSansBold12pt7b);

    String title = "Blood Pressure Monitor";
    int16_t titleWidth = tft.textWidth(title);
    tft.setCursor((SCREEN_WIDTH - titleWidth) / 2, 30);
    tft.println(title);

    tft.fillRect(topBoxX, topBoxY, boxWidth, boxHeight, TFT_WHITE);
    tft.drawRect(topBoxX, topBoxY, boxWidth, boxHeight, TFT_BLACK);

    tft.fillRect(topBoxX, bottomBoxY, boxWidth, boxHeight, TFT_WHITE);
    tft.drawRect(topBoxX, bottomBoxY, boxWidth, boxHeight, TFT_BLACK);

    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setFreeFont(&FreeSansBold9pt7b);

    int labelY = (boxHeight / 2) + 5;

    tft.setCursor(topBoxX + 10, topBoxY + labelY);
    tft.print("SYSTOLIC:");

    tft.setCursor(topBoxX + 10, bottomBoxY + labelY);
    tft.print("DIASTOLIC:");
}

void Display::updateValues(int systolic, int diastolic) {

    // Clear value areas only (fast refresh)
    tft.fillRect(topBoxX + 130, topBoxY + 1, 150, boxHeight - 2, TFT_WHITE);
    tft.fillRect(topBoxX + 130, bottomBoxY + 1, 150, boxHeight - 2, TFT_WHITE);

    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setFreeFont(&FreeSansBold18pt7b);

    int valueY = (boxHeight / 2) + 10;

    tft.setCursor(topBoxX + 140, topBoxY + valueY);
    tft.print(systolic);

    tft.setCursor(topBoxX + 140, bottomBoxY + valueY);
    tft.print(diastolic);
}

void Display::clear() {
    tft.fillScreen(TFT_BLACK);
}

void Display::showLines(const char* line1, const char* line2,
                        const char* line3, const char* line4) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFreeFont(&FreeSansBold9pt7b);

    int y = 40;
    if (line1) { tft.setCursor(10, y); tft.print(line1); y += 50; }
    if (line2) { tft.setCursor(10, y); tft.print(line2); y += 50; }
    if (line3) { tft.setCursor(10, y); tft.print(line3); y += 50; }
    if (line4) { tft.setCursor(10, y); tft.print(line4); }
}

void Display::testWhite() {
    tft.fillScreen(TFT_WHITE);
}