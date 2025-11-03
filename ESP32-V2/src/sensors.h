#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

class Adafruit_MPRLS;

// Initialize pressure sensor
bool initializePressureSensor(Adafruit_MPRLS *sensor);

// Read PPG sensor (analog)
int readPPGSensor();

// Read pressure sensor
float readPressureSensor(Adafruit_MPRLS *sensor);

// Convert absolute pressure to gauge pressure in mmHg
float convertToGaugePressure(float absolutePressure, float atmosphericPressure);

#endif