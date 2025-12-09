#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "BPMonitor.h"

// Abstract interface for logging - no Arduino dependency
class ILogger
{
public:
    virtual ~ILogger() {}
    virtual void log(const char* message) = 0;
    virtual void logLine(const char* message) = 0;
};

// Handles CSV formatting and logging
class DataLogger
{
private:
    ILogger* logger;
    BPMonitor* monitor;
    
public:
    DataLogger(ILogger* logger, BPMonitor* monitor);
    
    void printHeader();
    void printMeasurement(const BPMeasurement& measurement);
    void printComment(const char* comment);
    void printReport();  // Print detailed report at end of measurement
};

#endif