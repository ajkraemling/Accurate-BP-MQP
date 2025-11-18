#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "sensors.h"
#include "Display.h"
#include "BPMonitor.h"
#include "PulseDetector.h"

PressureSensor pressureSensor;
PPGSensor ppgSensor;
Display display;
BPMonitor bpMonitor;

// ============================================================================
// All detection algorithms
// ============================================================================

// Baseline Detectors (BL):
//   BL_W[window]_T[threshold]_D[minDev]_C[consecutive]
//   - W = Window size (number of samples for rolling average)      // 
//   - T = Threshold multiplier (standard deviations above mean)    // Seems 2.5 may be sweet spot? Needs more testing
//   - D = Minimum deviation (minimum signal change to detect)      // Doesn't seem to make a difference
//   - C = Consecutive readings required to confirm pulse
BaselineDetector det1(20, 2.5, 5, 2);
BaselineDetector det2(40, 2.5, 5, 2); // WAY TOO LOW
BaselineDetector det3(60, 2.5, 5, 2);
BaselineDetector det4(10, 2.5, 5, 2);
BaselineDetector det5(20, 1.0, 5, 2); // WAY TOO HIGH
BaselineDetector det6(20, 1.5, 5, 1); // TOO LOW
BaselineDetector det7(10, 1.0, 5, 1);
BaselineDetector det8(20, 2.5, 5, 1);
// BaselineDetector det9(40, 2.5, 10, 3);
// BaselineDetector det10(40, 2.5, 10, 1); // WAY TOO HIGH

// Derivative Detectors (DRV):
//   DRV_W[window]_T[threshold]
//   - W = Window size (samples to calculate derivative)
//   - T = Threshold (minimum rate of change to detect rising edge)
DerivativeDetector det11(5, 20); // WAY TOO LOW
DerivativeDetector det12(10, 20); // WAY TOO LOW
DerivativeDetector det13(5, 30); // WAY TOO HIGH
DerivativeDetector det14(5, 1); // WAY TOO HIGH

// Ensemble Detectors (ENS):
//   ENS_[votes]of[total]
//   - Voting system requiring N votes from M total detectors
// EnsembleDetector ensemble("ENS_3of5", 3);

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    delay(500);

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
    // bpMonitor.addDetector(&det9);
    // bpMonitor.addDetector(&det10);
    // bpMonitor.addDetector(&det11);
    // bpMonitor.addDetector(&det12);
    // bpMonitor.addDetector(&det13);
    // bpMonitor.addDetector(&det14);

    // Setup ensemble with best performing individual detectors
    // ensemble.addDetector(&det1);
    // ensemble.addDetector(&det3);
    // ensemble.addDetector(&det4);
    // ensemble.addDetector(&det7);
    // ensemble.addDetector(&det8);
    // bpMonitor.addDetector(&ensemble);

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