#include "sensors.h"
#include <Arduino.h>

PressureSensor::PressureSensor()
    : mpr(RESET_PIN, EOC_PIN), atmosphericPressure(0) {}

bool PressureSensor::begin()
{
    return mpr.begin();
}

void PressureSensor::calibrate(ILogger* logger)
{
    if (logger)
    {
        logger->logLine("Calibrating atmospheric pressure...");
    }
    
    float sum = 0;
    int samples = CALIBRATION_TIME_MS / 10;

    for (int i = 0; i < samples; i++)
    {
        sum += mpr.readPressure();
        delay(10);
    }

    atmosphericPressure = sum / samples;
    
    if (logger)
    {
        char buffer[64];
        sprintf(buffer, "Baseline Pressure: %.1f hPa", atmosphericPressure);
        logger->logLine(buffer);
        logger->logLine("Calibration complete!");
    }
}

float PressureSensor::readGaugePressure()
{
    float absolute = mpr.readPressure();
    return (absolute - atmosphericPressure) * HPA_TO_MMHG;
}

PPGSensor::PPGSensor(int pin)
    : filter(), analogPin(pin) {}

int PPGSensor::read()
{
    int raw = analogRead(analogPin);
    float filtered = filter.filter((float)raw);
    return (int)filtered;
}

int PPGSensor::readRaw()
{
    return analogRead(analogPin);
}

void PPGSensor::resetFilter()
{
    filter.reset();
}