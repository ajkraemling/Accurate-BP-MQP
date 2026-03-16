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

#include "MotorController.h"

// Declare objects 

PressureSensor pressureSensor; 
PPGSensor ppgSensor(PPG_PIN);

Display lcd;
DisplayPresenter presenter(&lcd);

BPMonitor bpMonitor;
PPGBandpassFilter ppgFilter(1000.0f / SAMPLE_RATE_MS);

MotorController motor;

MAPDetector mapDetector;

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

    Serial.println("Start");
    // sensors
    pressureSensor.begin();
    pressureSensor.calibrate();
    ppgSensor.resetFilter();

    // lcd display
    lcd.begin();
    presenter.showReady();

    // BP Monitor Setup 
    bpMonitor.setFilter(&ppgFilter);
    bpMonitor.setMAPDetector(&mapDetector);
    bpMonitor.setMotorController(&motor);

    // Motor set up
    bpMonitor.getMotorController()->begin();

    bpMonitor.reset();


    // Final detector decisions:
    // TIER ONE (BEST PERFORMING DETECTORS, 80 detectors) 
    int t1Windows[] = {60, 70, 80, 90, 100};
    float t1Thresh[] = {1.8, 1.9, 2.0, 2.2};
    int t1Dev[] = {5, 10, 12, 16};

    for (int w : t1Windows) {
        for (float t : t1Thresh) {
            for (int d : t1Dev) {
                Serial.print("Detector: ");
                Serial.print(w);
                Serial.print(" ");
                Serial.print(t);
                Serial.print(" ");
                Serial.println(d);

                bpMonitor.addDetector(new BaselineDetector(w, t, d));
            }
        }
    }

    // TIER TWO (HIGH WINDOW DETECTORS, 40 detectors) 
    int t2Windows[] = {110, 120, 130, 140, 150};
    float t2Thresh[] = {2.0, 2.4, 2.8, 3.3};
    int t2Dev[] = {12, 18};

    for (int w : t2Windows) {
        for (float t : t2Thresh) {
            for (int d : t2Dev) {
                Serial.print("Detector: ");
                Serial.print(w);
                Serial.print(" ");
                Serial.print(t);
                Serial.print(" ");
                Serial.println(d);

                bpMonitor.addDetector(new BaselineDetector(w, t, d));
            }
        }
    }

    // TIER THREE (LOW WINDOW DETECTORS, 24 detectors) 
    int t3Windows[] = {20, 30, 40, 50};
    float t3Thresh[] = {1.65, 1.8, 2.0};
    int t3Dev[] = {12, 15};

    for (int w : t3Windows) {
        for (float t : t3Thresh) {
            for (int d : t3Dev) {
                Serial.print("Detector: ");
                Serial.print(w);
                Serial.print(" ");
                Serial.print(t);
                Serial.print(" ");
                Serial.println(d);

                bpMonitor.addDetector(new BaselineDetector(w, t, d));
            }
        }
    }


    pinMode(13, INPUT_PULLUP);

    // Print headers
    Serial.println("Time,Pressure,rawPPGSignal,PPGSignal");
    // Start, we will put this in a loop and connect it to a button later
    bpMonitor.startInflation();
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

    // Update display screen
    presenter.showStatus(bpMonitor.getStatus());

    // Get auscultatory beat readings
    int button_not_pressed = digitalRead(13);

    // Serial outputs
    Serial.print(measurement.timestamp);
    Serial.print(",");
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.print(rawPPG);
    Serial.print(",");
    Serial.print(filteredPPG);
    Serial.print(",");
    Serial.println(button_not_pressed);

    // Complete function - State Machine status change 
    BPState currentState = bpMonitor.getState();

    if (lastState != COMPLETE && currentState == COMPLETE)
    {
        Serial.println("#SUMMARY_START");

        int detectorCount = bpMonitor.getDetectorCount();

        for (int i = 0; i < detectorCount; i++)
        {
            SystolicDetector* det = bpMonitor.getDetector(i);

            const char* name = det->getName();

            // pull all detections
            DetectionRecord detections[20];
            int count = 0;

            det->getTopDetections(detections, 20, &count);

            for (int k = 0; k < count; k++)
            {
                if (detections[k].confidence <= 0)
                    continue;

                Serial.print(name);
                Serial.print(",");
                Serial.print(detections[k].timestamp);
                Serial.print(",");
                Serial.print(detections[k].pressure, 0);
                Serial.print(",");
                Serial.println(detections[k].confidence, 3);
            }
        }

        Serial.println("Trying to get result");
        BPResult result = bpMonitor.getEnsembleResult();
        Serial.println("Got result");
        // Print Ensemble Result
        Serial.print("Ensemble,0,");
        Serial.print(result.systolic, 0);
        Serial.println(",0");

        // Print Oscillometric results
        MAPDetector* osc = bpMonitor.getMAPDetector();

        Serial.print("OscSys,0,");
        Serial.print(osc->getSystolic(), 0);
        Serial.println(",0");

        Serial.print("OscMAP,0,");
        Serial.print(osc->getMAP(), 0);
        Serial.println(",0");

        Serial.print("OscDia,0,");
        Serial.print(osc->getDiastolic(), 0);
        Serial.println(",0");

        // Get ensemble estimated diastolic
        float map = bpMonitor.getMAP();
        float DBP = (3.0f * map - result.systolic) / 2.0f;
        Serial.print("EstDia,0,");
        Serial.print(DBP, 0);
        Serial.println(",0");

        Serial.print("BPM,0,");
        Serial.print(bpMonitor.getBaselineBPM(), 0);
        Serial.println(",0");

        Serial.println("#SUMMARY_END");

        presenter.showStatus(bpMonitor.getStatus());

        // Complete measurement
        while (true) {
            delay(1000);
        }
    }

    lastState = currentState;

    delay(SAMPLE_RATE_MS);
}