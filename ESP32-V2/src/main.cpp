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

// ================= OBJECTS =================

PressureSensor pressureSensor;
PPGSensor ppgSensor(PPG_PIN);

Display lcd;
DisplayPresenter presenter(&lcd);

BPMonitor bpMonitor;
PPGBandpassFilter ppgFilter(1000.0f / SAMPLE_RATE_MS);

float lastMAP = 0;
float lastSys = 0;
float lastDia = 0;

int runNumber = 1;

// ================= MAIN CONTROL STATES =================

enum SystemState {
    SYS_CALIBRATING,
    SYS_READY,
    SYS_WAITING_FOR_PRESSURE,
    SYS_INFLATING,
    SYS_MEASURING,
    SYS_FINISHED
};

SystemState systemState = SYS_CALIBRATING;
unsigned long lastPulseTime = 0;
int pulsesAfterPeak = 0;
float maxPressureSeen = 0;
bool pastPeak = false;


// ================= SETUP =================

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    pressureSensor.begin();
    pressureSensor.calibrate();

    lcd.begin();
    presenter.showReady();

    // BP Monitor config
    bpMonitor.setFilter(&ppgFilter);

    int windows[] = {5, 10, 15, 20, 30, 40, 50, 60, 70, 80};
    float thresholds[] = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    int holds[] = {10};

    for (int w : windows)
        for (float t : thresholds)
            for (int h : holds)
                bpMonitor.addDetector(new BaselineDetector(w, t, h));

    bpMonitor.reset();

    Serial.println("Starting PPG calibration...");
}

// ================= LOOP =================

void loop()
{
    unsigned long now = millis();

    // =========================================================
    // 1️⃣ CALIBRATION PHASE
    // =========================================================
    if (systemState == SYS_CALIBRATING)
    {
        static unsigned long calibStart = millis();

        if (millis() - calibStart < 3000) {
            return;  // Let sensors settle, do nothing else
        }

        ppgSensor.resetFilter();   // Start clean
        Serial.println("Calibration complete");
        Serial.println("Ready - waiting for cuff inflation...");

        systemState = SYS_READY;
        return;
    }

    // =========================================================
    // 2️⃣ READY - WAIT FOR PRESSURE RISE
    // =========================================================
    if (systemState == SYS_READY)
    {
        float pressure = pressureSensor.readGaugePressure();

        if (pressure > 30)  // Cuff inflation detected
        {
            Serial.println("Pressure detected - starting measurement");
            systemState = SYS_WAITING_FOR_PRESSURE;
            maxPressureSeen = pressure;
        }

        delay(50);
        return;
    }

    // =========================================================
    // 3️⃣ WAIT FOR INFLATION PEAK
    // =========================================================
    if (systemState == SYS_WAITING_FOR_PRESSURE)
    {
        float pressure = pressureSensor.readGaugePressure();

        if (pressure > maxPressureSeen)
            maxPressureSeen = pressure;

        // Detect start of deflation
        if (pressure < maxPressureSeen - 5 && maxPressureSeen > 100)
        {
            Serial.println("Inflation complete - beginning measurement");
            Serial.println("Measuring during deflation...");

            Serial.print("Time,Pressure,PPGSignal,rawPPGSignal");
            for (int i = 0; i < bpMonitor.getDetectorCount(); i++) {
                Serial.print(",Det");
                Serial.print(i);
            }
            Serial.println();

            // Reset measurement tracking
            ppgSensor.resetFilter();
            bpMonitor.reset();

            lastPulseTime = millis();   // ✅ Prevent instant timeout
            pulsesAfterPeak = 0;

            systemState = SYS_MEASURING;
        }

        delay(20);
        return;
    }

    // =========================================================
    // 4️⃣ ACTIVE MEASUREMENT (ONLY PLACE PPG RUNS)
    // =========================================================
    if (systemState == SYS_MEASURING)
    {
        float pressure = pressureSensor.readGaugePressure();
        int rawPPG = ppgSensor.readRaw();
        int filteredPPG = ppgSensor.read();

        BPMeasurement measurement;
        measurement.pressure = pressure;
        measurement.timestamp = now;
        measurement.ppgSignal = filteredPPG;
        measurement.rawPPGSignal = rawPPG;

        bpMonitor.update(measurement);

        // CSV Output
        Serial.print(now); Serial.print(",");
        Serial.print(pressure, 2); Serial.print(",");
        Serial.print(filteredPPG); Serial.print(",");
        Serial.print(rawPPG);

        for (int i = 0; i < bpMonitor.getDetectorCount(); i++) {
            DetectionRecord best = bpMonitor.getDetector(i)->getBestDetection();
            Serial.print(",");
            Serial.print(best.pressure);
        }
        Serial.println();

        // ---- Pulse Tracking ----
        static int lastDetectionCount = 0;
        int currentDetectionCount = 0;

        for (int i = 0; i < bpMonitor.getDetectorCount(); i++) {
            currentDetectionCount += bpMonitor.getDetector(i)->getDetectionCount();
        }

        if (currentDetectionCount > lastDetectionCount) {
            lastPulseTime = now;

            if (pressure < maxPressureSeen - 20) {
                pulsesAfterPeak++;
                Serial.print("Deflation pulse detected (#");
                Serial.print(pulsesAfterPeak);
                Serial.print(") at pressure: ");
                Serial.println(pressure, 1);
            }

            lastDetectionCount = currentDetectionCount;
        }

        // ---- Stop Conditions ----
        bool hasEnoughDeflationPulses = (pulsesAfterPeak >= 3);
        bool pressureLow = (pressure < 50);
        bool noPulseTimeout = (now - lastPulseTime > 3000);

        static float maxOscAmp = 0;
        static float pressureAtMaxOsc = 0;
        static bool seenOscillations = false;
        static bool passedMAP = false;

        // Estimate oscillation amplitude
        float oscAmp = abs(filteredPPG);  // or filteredPPG - DC baseline

        if (oscAmp > 5)   // noise floor threshold (tune this)
            seenOscillations = true;

        if (oscAmp > maxOscAmp) {
            maxOscAmp = oscAmp;
            pressureAtMaxOsc = pressure;
        }

        // Detect falling side of curve (after MAP)
        if (seenOscillations && pressure < pressureAtMaxOsc - 10 && oscAmp < 0.5 * maxOscAmp) {
            passedMAP = true;
        }

        // FINAL STOP: oscillations mostly gone and cuff low
        if (passedMAP && oscAmp < 0.3 * maxOscAmp && pressure < 50)
        {
            Serial.println("Oscillation envelope captured — computing BP");

            MAPDetector* mapDetector = bpMonitor.getMAPDetector();
            bool mapSuccess = mapDetector->detectMAP();

            if (mapSuccess) {
                Serial.print("Systolic: ");
                Serial.println(mapDetector->getSystolic(), 1);
                Serial.print("MAP: ");
                Serial.println(mapDetector->getMAP(), 1);
                Serial.print("Diastolic: ");
                Serial.println(mapDetector->getDiastolic(), 1);
            }

            systemState = SYS_FINISHED;
        }

    }

    // =========================================================
    // 5️⃣ FINISHED — IDLE UNTIL CUFF REMOVED
    // =========================================================
    if (systemState == SYS_FINISHED)
    {
        float pressure = pressureSensor.readGaugePressure();

        if (pressure < 20)
        {
            Serial.println("\nReady for next measurement");
            Serial.println("Inflate cuff to begin...\n");

            bpMonitor.reset();
            ppgSensor.resetFilter();
            maxPressureSeen = 0;
            pulsesAfterPeak = 0;
            lastPulseTime = 0;

            systemState = SYS_READY;
        }

        delay(500);
        return;
    }
}
