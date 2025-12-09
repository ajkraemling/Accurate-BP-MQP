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
        logger->log(monitor->getDetector(i)->getName());
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

    // Detectors - show best detection pressure for each
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
    
    logger->logLine("");
    logger->logLine("=== Blood Pressure Measurement Report ===");
    
    snprintf(buffer, sizeof(buffer), "Max Pressure: %.1f mmHg", 
             monitor->getStatus().maxPressure);
    logger->logLine(buffer);
    logger->logLine("");
    
    // Find overall best
    float bestConfidence = 0;
    float bestPressure = monitor->getBestSystolic(&bestConfidence);
    
    snprintf(buffer, sizeof(buffer), "BEST READING: %.0f mmHg (confidence: %.2f)",
             bestPressure, bestConfidence);
    logger->logLine(buffer);
    logger->logLine("");
    
    logger->logLine("--- Detector Details ---");
    
    for (int i = 0; i < monitor->getDetectorCount(); i++)
    {
        SystolicDetector* det = monitor->getDetector(i);
        logger->logLine("");
        
        snprintf(buffer, sizeof(buffer), "%s:", det->getName());
        logger->logLine(buffer);
        
        snprintf(buffer, sizeof(buffer), "  Total detections: %d", 
                 det->getDetectionCount());
        logger->logLine(buffer);
        
        // Get top 3 detections
        DetectionRecord top[3];
        int topCount = 0;
        det->getTopDetections(top, 3, &topCount);
        
        if (topCount > 0)
        {
            logger->logLine("  Top detections:");
            for (int j = 0; j < topCount; j++)
            {
                snprintf(buffer, sizeof(buffer), 
                         "    #%d: %.0f mmHg @ %lums (conf: %.2f, %d consecutive beats)",
                         j + 1, top[j].pressure, top[j].timestamp,
                         top[j].confidence, top[j].subsequentBeats);
                logger->logLine(buffer);
            }
        }
        else
        {
            logger->logLine("  No valid detections");
        }
    }
    
    logger->logLine("");
    logger->logLine("========================================");
}