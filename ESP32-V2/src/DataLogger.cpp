#include "DataLogger.h"
#include <cstdio>  // for sprintf/snprintf

DataLogger::DataLogger(ILogger* logger, BPMonitor* monitor)
    : logger(logger), monitor(monitor) {}

void DataLogger::printHeader()
{
    logger->log("Time,Pressure,PPGSignal,rawPPGSignal");
    
    for (int i = 0; i < monitor->getDetectorCount(); i++)
    {
        logger->log(",");
        logger->log(monitor->getDetector(i)->getName());  // FIXED
    }
    
    logger->logLine("");
}

void DataLogger::printMeasurement(const BPMeasurement& measurement)
{
    char buffer[32];

    // Time
    snprintf(buffer, sizeof(buffer), "%lu", measurement.timestamp);
    logger->log(buffer);
    logger->log(",");

    // Pressure
    snprintf(buffer, sizeof(buffer), "%.2f", measurement.pressure); 
    logger->log(buffer);
    logger->log(",");

    // PPG
    snprintf(buffer, sizeof(buffer), "%d", measurement.ppgSignal);
    logger->log(buffer);
    logger->log(",");

    // Raw PPG
    snprintf(buffer, sizeof(buffer), "%d", measurement.rawPPGSignal);
    logger->log(buffer);

    // Detectors
    for (int i = 0; i < monitor->getDetectorCount(); i++)
    {
        logger->log(",");
        snprintf(buffer, sizeof(buffer), "%d",
                 monitor->getDetector(i)->getSystolic());
        logger->log(buffer);
    }

    logger->logLine("");
}

void DataLogger::printComment(const char* comment)
{
    logger->log("# ");
    logger->logLine(comment);
}
