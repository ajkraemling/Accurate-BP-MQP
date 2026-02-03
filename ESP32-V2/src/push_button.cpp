#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define LIGHT_BLUE 0x5D1F

int systolicValue = 1;
int diastolicValue = 1;

void drawMainPage() {
  tft.fillScreen(LIGHT_BLUE);
  
  tft.setTextColor(TFT_BLACK, LIGHT_BLUE);
  tft.setFreeFont(&FreeSansBold12pt7b);
  
  // Center the title
  String title = "Blood Pressure Monitor";
  int16_t titleWidth = tft.textWidth(title);
  tft.setCursor((SCREEN_WIDTH - titleWidth) / 2, 30);
  tft.println(title);
  
  int boxWidth = 280;
  int boxHeight = 70;
  int boxSpacing = 15;
  
  // Top box (Systolic)
  int topBoxX = 20;
  int topBoxY = 70;
  tft.fillRect(topBoxX, topBoxY, boxWidth, boxHeight, TFT_WHITE);
  tft.drawRect(topBoxX, topBoxY, boxWidth, boxHeight, TFT_BLACK);
  
  // Bottom box (Diastolic)
  int bottomBoxX = 20;
  int bottomBoxY = topBoxY + boxHeight + boxSpacing;
  tft.fillRect(bottomBoxX, bottomBoxY, boxWidth, boxHeight, TFT_WHITE);
  tft.drawRect(bottomBoxX, bottomBoxY, boxWidth, boxHeight, TFT_BLACK);
  
  // Draw labels (these don't change)
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setFreeFont(&FreeSansBold9pt7b);
  
  int labelY = (boxHeight / 2) + 5;
  
  tft.setCursor(topBoxX + 10, topBoxY + labelY);
  tft.print("SYSTOLIC:");
  
  tft.setCursor(bottomBoxX + 10, bottomBoxY + labelY);
  tft.print("DIASTOLIC:");
}

void updateValues() {
  int boxWidth = 280;
  int boxHeight = 70;
  int topBoxX = 20;
  int topBoxY = 70;
  int bottomBoxY = 155;
  
  // Clear only the number area (after the label)
  tft.fillRect(topBoxX + 130, topBoxY + 1, 150, boxHeight - 2, TFT_WHITE);
  tft.fillRect(topBoxX + 130, bottomBoxY + 1, 150, boxHeight - 2, TFT_WHITE);
  
  // Draw new values
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setFreeFont(&FreeSansBold18pt7b);
  
  int valueY = (boxHeight / 2) + 10;
  
  tft.setCursor(topBoxX + 140, topBoxY + valueY);
  tft.print(systolicValue);
  
  tft.setCursor(topBoxX + 140, bottomBoxY + valueY);
  tft.print(diastolicValue);
}

void setup() {
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.writecommand(0x36);
  tft.writedata(0xA0);
  
  drawMainPage();
  updateValues();
}

void loop() {
  delay(100);
  
  systolicValue++;
  if (systolicValue > 100) {
    systolicValue = 1;
  }
  
  diastolicValue++;
  if (diastolicValue > 100) {
    diastolicValue = 1;
  }
  
  updateValues();
}