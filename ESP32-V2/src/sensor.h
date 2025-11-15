#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "Adafruit_MPRLS.h"
#include "config.h"

class PressureSensor
{
private:
    Adafruit_MPRLS mpr;
    float atmosphericPressure;

public:
    PressureSensor();
    bool begin();
    void calibrate();
    float readGaugePressure();
};

class PPGSensor
{
public:
    PPGSensor();
    int read();
};

#endif