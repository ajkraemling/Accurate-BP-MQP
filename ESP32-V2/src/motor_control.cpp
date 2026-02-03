// #include <Arduino.h>
// #include <Wire.h>
// #include "Adafruit_MPRLS.h"
// #include "config.h"
// #include "sensors.h"
// #include "filtering.h"
// #include "beat_detection.h"
// #include "calibration.h"
// #include "signal_quality.h"
// #include "output.h"
// #include "lcd_display.h"
// #include <hd44780.h>
// #include <hd44780ioClass/hd44780_I2Cexp.h>

// // Global sensor objects
// Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
// hd44780_I2Cexp lcd(0x27);
// PulseFilterState pulseFilter;
// PressureFilterState pressureFilter;
// BeatDetectionState beatState;
// CalibrationData calibData;
// SignalQualityState qualityState;
// BPMeasurementData bpData;

// // Motor pins
// const int ENA = 23;
// const int IN1 = 17;
// const int IN2 = 16;
// const int a = 22;
// const int v = 21;

// const int pwmChan = 0;
// const int pwmFreq = 20000;
// const int pwmRes  = 8;

// void initializeMotor() {
//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(ENA, OUTPUT);
  
//   // Setup PWM on ENA pin
//   ledcSetup(pwmChan, pwmFreq, pwmRes);
//   ledcAttachPin(ENA, pwmChan);
  
//   // Start with motor off
//   ledcWrite(pwmChan, 0);
//   Serial.println("Motor initialized (OFF)");
// }

// void turnMotorOn(int pwmValue) {
//   // Blow air (forward direction)
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   ledcWrite(pwmChan, pwmValue);
//   Serial.print("Motor ON - PWM value: ");
//   Serial.println(pwmValue);
// }

// void turnMotorOff() {
//   ledcWrite(pwmChan, 0);
//   Serial.println("Motor OFF");
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(21, 22);
//   delay(1000);

//   printInitializationHeader();

//   if (!initializePressureSensor(&mpr))
//   {
//     Serial.println("ERROR: MPRLS sensor not found!");
//     while (1)
//       delay(10);
//   }
//   delay(100);

//   performCalibration(&calibData, &beatState, &mpr, &lcd);
  
//   // Initialize motor
//   initializeMotor();
  
//   // Turn motor on to pump air
//   turnMotorOn(200);  // 200/255 PWM = ~78% power
// }

// void loop() {
//   float rawPressure = readPressureSensor(&mpr);
//   float filteredPressure = applyPressureFilter(&pressureFilter, rawPressure);
//   float pressureGauge = convertToGaugePressure(filteredPressure, calibData.atmPressure);

//   Serial.print("Raw Pressure (hPa): ");
//   Serial.print(rawPressure, 2);
//   Serial.print(" | Filtered Pressure (hPa): ");
//   Serial.print(filteredPressure, 2);
//   Serial.print(" | Gauge Pressure (mmHg): ");
//   Serial.println(pressureGauge, 2);


  
//   delay(100);  // 100ms between readings

//   if (pressureGauge >= 100.0) {
//     // Target pressure reached, turn motor off
//     turnMotorOff();
//   }
//   else if (pressureGauge <= 100.0) {
//     // Target pressure reached, turn motor off
//     turnMotorOn(200);  // Set PWM to 0 to stop motor
//   }
// }