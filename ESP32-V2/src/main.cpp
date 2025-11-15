#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include "config.h"
#include "sensors.h"
#include "filtering.h"
#include "beat_detection.h"
#include "calibration.h"
#include "signal_quality.h"
#include "output.h"
#include "lcd_display.h"
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// Global sensor objects
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
hd44780_I2Cexp lcd(0x27);
PulseFilterState pulseFilter;
PressureFilterState pressureFilter;
BeatDetectionState beatState;
CalibrationData calibData;
SignalQualityState qualityState;
BPMeasurementData bpData;

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(1000);

  printInitializationHeader();

  if (!initializePressureSensor(&mpr))
  {
    Serial.println("ERROR: MPRLS sensor not found!");
    while (1)
      delay(10);
  }

  if (initializeLCD(&lcd) && DISPLAY_MODE)
  {
    Serial.println("ERROR: LCD display not found!");
    while (1)
      delay(10);
  }

  initializeFilters(&pulseFilter, &pressureFilter);
  initializeBPMeasurement(&bpData);
  performCalibration(&calibData, &beatState, &mpr, &lcd);
  printOutputHeader();
}

void loop()
{
  // Read sensors
  int rawPPGSignal = readPPGSensor();
  int filteredSignal = applyPulseFilter(&pulseFilter, rawPPGSignal);

  float rawPressure = readPressureSensor(&mpr);
  float filteredPressure = applyPressureFilter(&pressureFilter, rawPressure);

  if (!isPressureInitialized(&pressureFilter))
  {
    delay(SAMPLE_DELAY_MS);
    return;
  }

  float pressureGauge = convertToGaugePressure(filteredPressure, calibData.atmPressure);
  unsigned long currentTime = millis();

  // Update adaptive thresholds
  updateThresholds(&calibData, &beatState, filteredSignal, currentTime);

  // Switch to sensitive mode during BP measurement
  // bool shouldUseSensitiveMode = (bpData.state == MEASURE_SYSTOLIC || bpData.state == MEASURE_DIASTOLIC);
  // setBPMeasurementMode(&beatState, shouldUseSensitiveMode, calibData.minSignal, calibData.maxSignal);

  // Detect heartbeat
  bool heartbeatOccurred = detectHeartbeat(&beatState, filteredSignal, currentTime);

  // Update blood pressure measurement
  updateBPMeasurement(&bpData, pressureGauge, rawPPGSignal, &lcd);

  // Check signal quality periodically and print to LCD display
  // checkQuality(&lcd, &qualityState, &calibData, &beatState, filteredSignal, currentTime);

  // Output data
  outputData(heartbeatOccurred, beatState.beatDetected, rawPPGSignal,
             filteredSignal, pressureGauge, currentTime, &bpData);

  delay(SAMPLE_DELAY_MS);
}
