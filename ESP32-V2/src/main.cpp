#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "Sensor.h"
#include "Display.h"
#include "BPMonitor.h"
#include "PulseDetector.h"

PressureSensor pressureSensor;
PPGSensor ppgSensor;
Display display;
BPMonitor bpMonitor;

// ============================================================================
// DETECTION ALGORITHM NAMING CONVENTION
// ============================================================================
// Format: TYPE_PARAM1_PARAM2_...
//
// Baseline Detectors (BL):
//   BL_W[window]_T[threshold]_D[minDev]_C[consecutive]
//   - W = Window size (number of samples for rolling average)
//   - T = Threshold multiplier (standard deviations above mean)
//   - D = Minimum deviation (minimum signal change to detect)
//   - C = Consecutive readings required to confirm pulse
//
// Derivative Detectors (DRV):
//   DRV_W[window]_T[threshold]
//   - W = Window size (samples to calculate derivative)
//   - T = Threshold (minimum rate of change to detect rising edge)
//
// Threshold Detectors (THR):
//   THR_[value]
//   - Simple fixed threshold crossing detection
//
// Ensemble Detectors (ENS):
//   ENS_[votes]of[total]
//   - Voting system requiring N votes from M total detectors
// ============================================================================

// All our detection algorithms
BaselineDetector det1("BL_W40_T2.5_D10_C2", 40, 2.5, 10, 2);
BaselineDetector det2("BL_W60_T2.5_D10_C2", 60, 2.5, 10, 2);
BaselineDetector det3("BL_W40_T3.0_D10_C2", 40, 3.0, 10, 2);
BaselineDetector det4("BL_W40_T2.0_D10_C2", 40, 2.0, 10, 2);
BaselineDetector det5("BL_W40_T2.5_D15_C2", 40, 2.5, 15, 2);
BaselineDetector det6("BL_W40_T2.5_D5_C2", 40, 2.5, 5, 2);
BaselineDetector det7("BL_W40_T2.5_D10_C3", 40, 2.5, 10, 3);
BaselineDetector det8("BL_W40_T2.5_D10_C1", 40, 2.5, 10, 1);
BaselineDetector det9("BL_W20_T2.5_D10_C2", 20, 2.5, 10, 2);
BaselineDetector det10("BL_W80_T2.5_D10_C2", 80, 2.5, 10, 2);

DerivativeDetector det11("DRV_W5_T20", 5, 20);
DerivativeDetector det12("DRV_W10_T20", 10, 20);
DerivativeDetector det13("DRV_W5_T30", 5, 30);
DerivativeDetector det14("DRV_W5_T10", 5, 10);

ThresholdDetector det15("THR_100", 100);
ThresholdDetector det16("THR_150", 150);
ThresholdDetector det17("THR_200", 200);

// Ensemble detector
EnsembleDetector ensemble("ENS_3of5", 3);

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    delay(1000);

    Serial.println("\n========== Blood Pressure Monitor ==========");

    // Initialize pressure sensor
    if (!pressureSensor.begin())
    {
        Serial.println("ERROR: Pressure sensor not found!");
        while (1)
            delay(10);
    }

    // Initialize display
    if (!display.begin())
    {
        Serial.println("ERROR: LCD not found!");
        while (1)
            delay(10);
    }

    // Add all detectors to BP monitor
    bpMonitor.addDetector(&det1);
    bpMonitor.addDetector(&det2);
    bpMonitor.addDetector(&det3);
    bpMonitor.addDetector(&det4);
    bpMonitor.addDetector(&det5);
    bpMonitor.addDetector(&det6);
    bpMonitor.addDetector(&det7);
    bpMonitor.addDetector(&det8);
    bpMonitor.addDetector(&det9);
    bpMonitor.addDetector(&det10);
    bpMonitor.addDetector(&det11);
    bpMonitor.addDetector(&det12);
    bpMonitor.addDetector(&det13);
    bpMonitor.addDetector(&det14);
    bpMonitor.addDetector(&det15);
    bpMonitor.addDetector(&det16);
    bpMonitor.addDetector(&det17);

    // Setup ensemble with best performing individual detectors
    ensemble.addDetector(&det1);
    ensemble.addDetector(&det3);
    ensemble.addDetector(&det11);
    ensemble.addDetector(&det12);
    ensemble.addDetector(&det16);
    bpMonitor.addDetector(&ensemble);

    // Countdown and calibration
    for (int i = 5; i > 0; i--)
    {
        display.showCountdown(i);
        Serial.print(i);
        Serial.print("... ");
        delay(1000);
    }
    Serial.println();

    display.print("Calibrating...");
    pressureSensor.calibrate();

    display.print("Ready!");
    delay(1000);

    Serial.println("\n========== Ready for Measurement ==========");
    // Print CSV header
    bpMonitor.printCSVHeader();
}

void loop()
{
    int ppgSignal = ppgSensor.read();
    float pressure = pressureSensor.readGaugePressure();

    bpMonitor.update(pressure, ppgSignal, display);
    bpMonitor.printCSVRow(pressure, ppgSignal);

    delay(SAMPLE_RATE_MS);
}