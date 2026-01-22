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

/* =======================
   Hardware
   ======================= */
PressureSensor pressureSensor;
PPGSensor      ppgSensor;
Display        display;
SerialLogger   serialLogger;

/* =======================
   Presentation
   ======================= */
DisplayPresenter presenter(&display);

/* =======================
   BP Processing
   ======================= */
BPMonitor bpMonitor;

/* --- Detectors copied from test_main.cpp logic --- */
BaselineDetector det1(25, 2.0, 10);
BaselineDetector det2(30, 2.2, 10);
BaselineDetector det3(35, 2.4, 10);
BaselineDetector det4(40, 2.6, 10);

/* PPG filter */
PPGBandpassFilter ppgFilter(1000.0f / SAMPLE_RATE_MS);

/* =======================
   State
   ======================= */
bool measurementLocked = false;

/* =======================
   Setup
   ======================= */
void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    delay(500);

    serialLogger.logLine("\n=== BP Monitor (REAL-TIME) ===");

    if (!pressureSensor.begin()) {
        presenter.showError("Pressure sensor");
        while (1);
    }

    if (!display.begin()) {
        serialLogger.logLine("Display error");
        while (1);
    }

    /* Add detectors (same as test_main) */
    bpMonitor.addDetector(&det1);
    bpMonitor.addDetector(&det2);
    bpMonitor.addDetector(&det3);
    bpMonitor.addDetector(&det4);

    pressureSensor.calibrate(&serialLogger);

    presenter.showReady();
}


void loop()
{
    /* Always read sensors */
    float pressure = pressureSensor.readGaugePressure();
    int rawPPG     = ppgSensor.readRaw();
    float filteredPPG = ppgFilter.filter((float)rawPPG);

    BPMeasurement m;
    m.pressure      = pressure;
    m.ppgSignal     = (int)filteredPPG;
    m.rawPPGSignal  = rawPPG;
    m.timestamp     = millis();

    /* Run detection only until locked */
    if (!measurementLocked) {
        bpMonitor.update(m);
    }

    BPStatus status = bpMonitor.getStatus();

    /* LCD output */
    presenter.showStatus(status);

    /* Serial output (real-time) */
    Serial.print("P=");
    Serial.print(pressure, 1);
    Serial.print(" ");

    float map = bpMonitor.getMAP();
    if (map > 0) {
        Serial.print("MAP=");
        Serial.print(map, 0);
        Serial.print(" ");
    }

    BPResult ensemble = bpMonitor.getEnsembleResult();
    if (ensemble.systolic > 0) {
        Serial.print("SBP=");
        Serial.print(ensemble.systolic, 0);
        Serial.print(" Conf=");
        Serial.print(ensemble.confidence, 3);
    }

    Serial.println();

    /* ==========================
       LOCK ONCE STABLE (Option B)
       ========================== */
    if (!measurementLocked && status.state == COMPLETE) {
        measurementLocked = true;

        Serial.println("\n=== MEASUREMENT LOCKED ===");
        Serial.print("Final SBP: ");
        Serial.println(ensemble.systolic, 0);
        Serial.print("MAP: ");
        Serial.println(map, 0);

        /*
         * NOTE: CHECK WHEN TESTING
         * Systolic detection is set to 0.75 oscillometric ratio.
         * This value is random --> FINETUNING NEEDED! ;)
         */
    }

    delay(SAMPLE_RATE_MS);
}
