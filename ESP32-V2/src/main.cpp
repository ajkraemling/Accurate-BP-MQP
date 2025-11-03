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

// Global sensor objects
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
PulseFilterState pulseFilter;
PressureFilterState pressureFilter;
BeatDetectionState beatState;
CalibrationData calibData;
SignalQualityState qualityState;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  printInitializationHeader();

  if (!initializePressureSensor(&mpr))
  {
    Serial.println("ERROR: MPRLS sensor not found!");
    while (1)
      delay(10);
  }

  initializeFilters(&pulseFilter, &pressureFilter);
  performCalibration(&calibData, &beatState, mpr);
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

  // Detect heartbeat
  bool heartbeatOccurred = detectHeartbeat(&beatState, filteredSignal, currentTime);

  // Check signal quality periodically
  checkQuality(&qualityState, &calibData, &beatState, filteredSignal, currentTime);

  // Output data
  outputData(heartbeatOccurred, beatState.beatDetected, rawPPGSignal,
             filteredSignal, pressureGauge, currentTime);

  delay(SAMPLE_DELAY_MS);
}