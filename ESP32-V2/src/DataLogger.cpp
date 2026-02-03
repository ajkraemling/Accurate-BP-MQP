#include "DataLogger.h"
#include <cstdio>

DataLogger::DataLogger(ILogger* logger, BPMonitor* monitor)
    : logger(logger), monitor(monitor) {}

void DataLogger::printHeader()
{
    logger->log("Time,Pressure,PPGSignal,rawPPGSignal");

    for (int i = 0; i < monitor->getDetectorCount(); i++)
    {
        logger->log(",");
        logger->log(monitor->getDetector(i)->getName());
    }
    logger->logLine("");
}

void DataLogger::printMeasurement(const BPMeasurement& m)
{
    char buffer[32];

    snprintf(buffer, sizeof(buffer), "%lu", m.timestamp);
    logger->log(buffer); logger->log(",");

    snprintf(buffer, sizeof(buffer), "%.2f", m.pressure);
    logger->log(buffer); logger->log(",");

    snprintf(buffer, sizeof(buffer), "%d", m.ppgSignal);
    logger->log(buffer); logger->log(",");

    snprintf(buffer, sizeof(buffer), "%d", m.rawPPGSignal);
    logger->log(buffer);

    for (int i = 0; i < monitor->getDetectorCount(); i++)
    {
        logger->log(",");
        DetectionRecord best = monitor->getDetector(i)->getBestDetection();
        snprintf(buffer, sizeof(buffer), "%.0f", best.pressure);
        logger->log(buffer);
    }

    logger->logLine("");
}

void DataLogger::printComment(const char* comment)
{
    logger->log("# ");
    logger->logLine(comment);
}

void DataLogger::printReport()
{
    char buffer[128];

    logger->logLine("\n=== BP Measurement Report ===");

    float sys = monitor->getSystolic();
    float map = monitor->getMAP();

    snprintf(buffer, sizeof(buffer), "Systolic: %.0f mmHg", sys);
    logger->logLine(buffer);

    snprintf(buffer, sizeof(buffer), "MAP: %.0f mmHg", map);
    logger->logLine(buffer);

    logger->logLine("=============================");
}
