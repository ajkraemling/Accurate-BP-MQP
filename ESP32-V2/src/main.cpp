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
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    Wire.begin();

    Serial.println("Start");

    // sensors
    pressureSensor.begin();
    pressureSensor.calibrate();
    ppgSensor.resetFilter();

    // BP Monitor Setup 
    bpMonitor.setFilter(&ppgFilter);
    bpMonitor.setMAPDetector(&mapDetector);
    bpMonitor.setMotorController(&motor);

    // Motor set up
    bpMonitor.getMotorController()->begin();

    int windows[] = {5, 10, 15, 20, 30, 40, 50, 60, 70, 80};
    float thresholds[] = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    int holds[] = {10};

    for (int w : windows) {
        for (float t : thresholds) {
            for (int h : holds) {
                Serial.print("Detector: ");
                Serial.print(w);
                Serial.print(" ");
                Serial.print(t);
                Serial.print(" ");
                Serial.println(h);

                bpMonitor.addDetector(new BaselineDetector(w, t, h));
            }
        }
    }

    bpMonitor.reset();

    pinMode(13, INPUT_PULLUP);

    Serial.println("Time,Pressure,rawPPGSignal,PPGSignal");
    bpMonitor.startInflation();
}

void loop()
{
    // Read Sensors
    float pressure = pressureSensor.readGaugePressure();
    int rawPPG = ppgSensor.readRaw();
    int filteredPPG = ppgSensor.read();

    BPMeasurement measurement;
    measurement.pressure = pressure;
    measurement.timestamp = millis();
    measurement.ppgSignal = filteredPPG;
    measurement.rawPPGSignal = rawPPG;

    // Update sensors
    bpMonitor.update(measurement);

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

        Serial.print("Ensemble,0,");
        Serial.print(result.systolic, 0);
        Serial.println(",0");

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

        float map = bpMonitor.getMAP();
        float DBP = (3.0f * map - result.systolic) / 2.0f;

        Serial.print("EstDia,0,");
        Serial.print(DBP, 0);
        Serial.println(",0");

        Serial.print("BPM,0,");
        Serial.print(bpMonitor.getBaselineBPM(), 0);
        Serial.println(",0");

        Serial.println("#SUMMARY_END");

        // Send BP results to screen ESP32
        Serial2.print((int)result.systolic);
        Serial2.print(",");
        Serial2.println((int)DBP);

        // Complete measurement
        while (true) {
            delay(1000);
        }
    }

    lastState = currentState;

    delay(SAMPLE_RATE_MS);
}



///////////////// -- Code for Display ESP32 -- /////////////////

// #include <Arduino.h>
// #include <Wire.h>
// #include "display.h"
// #include "DisplayPresenter.h"

// Display lcd;
// DisplayPresenter presenter(&lcd);

// void setup()
// {
//     Serial.begin(115200);
//     Serial2.begin(115200, SERIAL_8N1, 16, 17);
//     Wire.begin();

//     lcd.begin();
//     lcd.drawMainPage();
// }

// void loop()
// {
//     if (Serial2.available()) {
//         String data = Serial2.readStringUntil('\n');
//         int commaIndex = data.indexOf(',');
//         if (commaIndex != -1) {
//             int systolic = data.substring(0, commaIndex).toInt();
//             int diastolic = data.substring(commaIndex + 1).toInt();
//             lcd.updateValues(systolic, diastolic);
//         }
//     }
// }