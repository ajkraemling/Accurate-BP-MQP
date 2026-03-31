// ///////////////// -- Code for Display ESP32 -- /////////////////

// #include <Arduino.h>
// #include <Wire.h>
// #include "display.h"
// #include "DisplayPresenter.h"

// Display lcd;
// DisplayPresenter presenter(&lcd);

// void setup()
// {
//     Serial.begin(115200);
//     Serial2.begin(115200, SERIAL_8N1, 16, 17);
//     Wire.begin();

//     lcd.begin();
//     lcd.drawMainPage();
// }

// void loop()
// {
//     if (Serial2.available()) {
//         String data = Serial2.readStringUntil('\n');
//         int commaIndex = data.indexOf(',');
//         if (commaIndex != -1) {
//             int systolic = data.substring(0, commaIndex).toInt();
//             int diastolic = data.substring(commaIndex + 1).toInt();
//             lcd.updateValues(systolic, diastolic);
//         }
//     }
// }