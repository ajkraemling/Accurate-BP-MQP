#include "sensors.h"
#include "config.h"
#include "Adafruit_MPRLS.h"

bool initializePressureSensor(Adafruit_MPRLS *sensor)
{
    return sensor->begin();
}

int readPPGSensor()
{
    return analogRead(PULSESENSOR_OUT);
}

float readPressureSensor(Adafruit_MPRLS *sensor)
{
    return sensor->readPressure();
}

float convertToGaugePressure(float absolutePressure, float atmosphericPressure)
{
    return (absolutePressure - atmosphericPressure) * HPA_TO_MMHG;
}