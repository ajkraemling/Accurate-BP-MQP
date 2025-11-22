#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H

#include <Arduino.h>
#include "DataLogger.h"

// Arduino Serial implementation of ILogger
class SerialLogger : public ILogger
{
public:
    void log(const char* message) override;
    void logLine(const char* message) override;
};

#endif