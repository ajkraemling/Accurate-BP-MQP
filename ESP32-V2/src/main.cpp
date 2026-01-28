#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensors.h"
#include "display.h"
#include "DisplayPresenter.h"
#include "SerialLogger.h"
#include "DataLogger.h"

#include "BPMonitor.h"
#include "SystolicDetector.h"
#include "MAPDetector.h"
#include "filters.h"

// Declare objects 

PressureSensor pressureSensor;
PPGSensor ppgSensor(PPG_PIN);

Display lcd;
DisplayPresenter presenter(&lcd);

BPMonitor bpMonitor;
PPGBandpassFilter ppgFilter(1000.0f / SAMPLE_RATE_MS);

float lastMAP = 0;
float lastSys = 0;
float lastDia = 0;

BPState lastState = IDLE;
int runNumber = 1;

// Setup

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // sensors
    pressureSensor.begin();
    pressureSensor.calibrate();
    ppgSensor.resetFilter();

    // lcd display
    lcd.begin();
    presenter.showReady();

    // BP Monitor Setup 
    bpMonitor.setFilter(&ppgFilter);

    int windows[] = {5, 10, 15, 20, 30, 40, 50, 60, 70, 80};
    float thresholds[] = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    int holds[] = {10};

    for (int w : windows) {
        for (float t : thresholds) {
            for (int h : holds) {
                bpMonitor.addDetector(new BaselineDetector(w, t, h));
            }
        }
    }

    Serial.print("Detectors active: ");
    Serial.println(bpMonitor.getDetectorCount());

    bpMonitor.reset();
}


void loop()
{
    // Read Sensors
    float pressure = pressureSensor.readGaugePressure();
    int rawPPG = ppgSensor.readRaw();
    int filteredPPG = ppgSensor.read();  // already bandpass filtered

    BPMeasurement measurement;
    measurement.pressure = pressure;
    measurement.timestamp = millis();
    measurement.ppgSignal = filteredPPG;
    measurement.rawPPGSignal = rawPPG;

    // Update sensors
    bpMonitor.update(measurement);

    //Oscillmetric tracking
    float oscAmp = bpMonitor.getMAPDetector()->getLatestAmplitude();

    float mapP = bpMonitor.getMAPDetector()->getMAP();
    float sysP = bpMonitor.getMAPDetector()->getSystolic();
    float diaP = bpMonitor.getMAPDetector()->getDiastolic();

    if (mapP > 0 && pressure <= mapP) lastMAP = mapP;
    if (sysP > 0 && pressure <= sysP) lastSys = sysP;
    if (diaP > 0 && pressure <= diaP) lastDia = diaP;

    // Update display screen
    BPStatus status = bpMonitor.getStatus();
    presenter.showStatus(status);

    //serial outputs (dor debugging)
    Serial.print("P=");
    Serial.print(pressure, 1);
    Serial.print("  Osc=");
    Serial.print(oscAmp, 2);
    Serial.print("  MAP=");
    Serial.print(lastMAP, 0);
    Serial.print("  Sys=");
    Serial.print(lastSys, 0);
    Serial.print("  Dia=");
    Serial.print(lastDia, 0);

    BPResult ensemble = bpMonitor.getEnsembleResult();
    if (ensemble.systolic > 0) {
        Serial.print("  Ensemble=");
        Serial.print(ensemble.systolic, 0);
        Serial.print("  Conf=");
        Serial.print(ensemble.confidence, 3);
    }
    Serial.println();

    //Complete function - State Machine status change 
    BPState currentState = bpMonitor.getState();

    if (lastState != COMPLETE && currentState == COMPLETE)
    {
        Serial.println("\n===== MEASUREMENT COMPLETE =====");

        float map = bpMonitor.getMAP();
        BPResult result = bpMonitor.getEnsembleResult();

        Serial.print("Run #");
        Serial.println(runNumber++);
        Serial.print("Final Systolic: ");
        Serial.println(result.systolic, 0);
        Serial.print("MAP: ");
        Serial.println(map, 0);

        if (result.systolic > 0 && map > 0) {
            float DBP = (3.0f * map - result.systolic) / 2.0f;
            Serial.print("Estimated Diastolic: ");
            Serial.println(DBP, 0);
        }

        Serial.println("================================");

        presenter.showStatus(bpMonitor.getStatus());


        while (true) {
            delay(1000);
        }
    }

    lastState = currentState;

    delay(SAMPLE_RATE_MS);
}
