#include "SerialLogger.h"

void SerialLogger::log(const char* message)
{
    Serial.print(message);
}

void SerialLogger::logLine(const char* message)
{
    Serial.println(message);
}