#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_MPRLS.h>
#include "config.h"
#include "filters.h"
#include "DataLogger.h"

class PressureSensor {
private:
    Adafruit_MPRLS mpr;
    float atmosphericPressure;

public:
    PressureSensor();
    bool begin();
    void calibrate(ILogger* logger = nullptr);
    float readGaugePressure();
};

class PPGSensor {
private:
    PPGBandpassFilter filter;
    int analogPin;
    
public:
    PPGSensor(int pin = PPG_PIN);
    
    // Read filtered PPG signal
    int read();
    
    // Read raw unfiltered signal (for debugging)
    int readRaw();
    
    // Reset filter state
    void resetFilter();
};

#endif