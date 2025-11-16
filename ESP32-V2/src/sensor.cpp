#include "sensor.h"

PressureSensor::PressureSensor()
    : mpr(RESET_PIN, EOC_PIN), atmosphericPressure(0) {}

bool PressureSensor::begin()
{
    return mpr.begin();
}

void PressureSensor::calibrate()
{
    Serial.println("Calibrating atmospheric pressure...");
    float sum = 0;
    int samples = CALIBRATION_TIME_MS / 10;

    for (int i = 0; i < samples; i++)
    {
        sum += mpr.readPressure();
        delay(10);
    }

    atmosphericPressure = sum / samples;
    Serial.print("Baseline Pressure: ");
    Serial.print(atmosphericPressure, 1);
    Serial.println(" hPa");
    Serial.println("Calibration complete!");
}

float PressureSensor::readGaugePressure()
{
    float absolute = mpr.readPressure();
    return (absolute - atmosphericPressure) * HPA_TO_MMHG;
}

PPGSensor::PPGSensor() {}

int PPGSensor::read()
{
    return analogRead(PPG_PIN);
}