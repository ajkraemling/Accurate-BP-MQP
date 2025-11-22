#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "sensors.h"
#include "Display.h"
#include "DisplayPresenter.h"
#include "SerialLogger.h"
#include "DataLogger.h"
#include "BPMonitor.h"
#include "PulseDetector.h"

// Hardware adapters
PressureSensor pressureSensor;
PPGSensor ppgSensor;
Display display;
SerialLogger serialLogger;

// Presenters and business logic
DisplayPresenter presenter(&display);
BPMonitor bpMonitor;
DataLogger dataLogger(&serialLogger, &bpMonitor);

// Detection algorithms
BaselineDetector det1(20, 2.5, 5, 2);
BaselineDetector det2(40, 2.5, 5, 2);
BaselineDetector det3(60, 2.5, 5, 2);
BaselineDetector det4(10, 2.5, 5, 2);
BaselineDetector det5(20, 1.0, 5, 2);
BaselineDetector det6(20, 1.5, 5, 1);
BaselineDetector det7(10, 1.0, 5, 1);
BaselineDetector det8(20, 2.5, 5, 1);

DerivativeDetector det11(5, 20);
DerivativeDetector det12(10, 20);
DerivativeDetector det13(5, 30);
DerivativeDetector det14(5, 1);

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    delay(500);

    serialLogger.logLine("\n========== Blood Pressure Monitor ==========");

    // Initialize pressure sensor
    if (!pressureSensor.begin())
    {
        serialLogger.logLine("ERROR: Pressure sensor not found!");
        presenter.showError("Pressure sensor");
        while (1)
            delay(10);
    }

    // Initialize display
    if (!display.begin())
    {
        serialLogger.logLine("ERROR: LCD not found!");
        while (1)
            delay(10);
    }

    // Add detectors to BP monitor
    bpMonitor.addDetector(&det1);
    bpMonitor.addDetector(&det2);
    bpMonitor.addDetector(&det3);
    bpMonitor.addDetector(&det4);
    bpMonitor.addDetector(&det5);
    bpMonitor.addDetector(&det6);
    bpMonitor.addDetector(&det7);
    bpMonitor.addDetector(&det8);

    // Countdown
    for (int i = 5; i > 0; i--)
    {
        presenter.showCountdown(i);
        char buffer[16];
        sprintf(buffer, "%d... ", i);
        serialLogger.log(buffer);
        delay(1000);
    }
    serialLogger.logLine("");

    // Calibration
    presenter.showCalibrating();
    pressureSensor.calibrate(&serialLogger);

    presenter.showReady();
    delay(1000);

    serialLogger.logLine("\n========== Ready for Measurement ==========");
    dataLogger.printHeader();
}

void loop()
{
    // Read sensors
    int ppgSignal = ppgSensor.read();
    int rawPPGSignal = ppgSensor.readRaw();
    float pressure = pressureSensor.readGaugePressure();
    unsigned long timestamp = millis();

    // Create measurement
    BPMeasurement measurement;
    measurement.pressure = pressure;
    measurement.ppgSignal = ppgSignal;
    measurement.rawPPGSignal = rawPPGSignal;
    measurement.timestamp = timestamp;

    // Update business logic
    bpMonitor.update(measurement);
    
    // Update UI
    BPStatus status = bpMonitor.getStatus();
    presenter.showStatus(status);
    
    // Log data
    dataLogger.printMeasurement(measurement);
    
    // Check for timeout
    if (status.state == COMPLETE)
    {
        dataLogger.printComment("Measurement complete or timeout");
    }

    delay(SAMPLE_RATE_MS);
}