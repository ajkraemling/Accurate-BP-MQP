#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_MPRLS.h>
#include "config.h"
#include "filters.h"

class PressureSensor {
private:
    Adafruit_MPRLS mpr;
    float atmosphericPressure;

public:
    PressureSensor();
    bool begin();
    void calibrate();
    float readGaugePressure();
};

class PPGSensor {
private:
    PPGBandpassFilter filter;
    
public:
    PPGSensor();
    
    // Constructor with custom sample rate
    PPGSensor(int sampleRate);
    
    // Read filtered PPG signal
    int read();
    
    // Read raw unfiltered signal (for debugging)
    int readRaw();
    
    // Reset filter state
    void resetFilter();
};

#endif