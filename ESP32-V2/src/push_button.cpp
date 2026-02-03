#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define LIGHT_BLUE 0x5D1F

void setup() {
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.writecommand(0x36);
  tft.writedata(0xA0);
  
  tft.fillScreen(LIGHT_BLUE);
  
  tft.setTextColor(TFT_BLACK, LIGHT_BLUE);
  tft.setFreeFont(&FreeSans12pt7b);  // Smooth font for title
  tft.setCursor(20, 30);
  tft.println("Blood Pressure Monitor");
  
  int boxWidth = 280;
  int boxHeight = 70;
  int boxSpacing = 15;
  
  int topBoxX = 20;
  int topBoxY = 70;
  tft.fillRect(topBoxX, topBoxY, boxWidth, boxHeight, TFT_WHITE);
  tft.drawRect(topBoxX, topBoxY, boxWidth, boxHeight, TFT_BLACK);
  
  int bottomBoxX = 20;
  int bottomBoxY = topBoxY + boxHeight + boxSpacing;
  tft.fillRect(bottomBoxX, bottomBoxY, boxWidth, boxHeight, TFT_WHITE);
  tft.drawRect(bottomBoxX, bottomBoxY, boxWidth, boxHeight, TFT_BLACK);
  
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setFreeFont(&FreeSans9pt7b);  // Smooth font for labels
  
  tft.setCursor(topBoxX + 10, topBoxY + 25);
  tft.println("SYSTOLIC:");
  
  tft.setCursor(bottomBoxX + 10, bottomBoxY + 25);
  tft.println("DIASTOLIC:");
}

void loop() {
  // Main loop does nothing; display is static for now
}