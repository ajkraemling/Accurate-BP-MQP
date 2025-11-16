#include "BPMonitor.h"

BPMonitor::BPMonitor()
    : state(IDLE), systolic(0), maxPressure(0), startTime(0),
      detectorCount(0), detectionCount(0)
{
    for (int i = 0; i < MAX_DETECTORS; i++)
    {
        detectors[i] = nullptr;
    }
}

BPMonitor::~BPMonitor()
{
    // Detectors are managed externally
}

void BPMonitor::addDetector(PulseDetector *detector)
{
    if (detectorCount < MAX_DETECTORS)
    {
        detectors[detectorCount++] = detector;
    }
}

void BPMonitor::reset()
{
    state = IDLE;
    systolic = 0;
    maxPressure = 0;
    startTime = 0;
    detectionCount = 0;
    for (int i = 0; i < detectorCount; i++)
    {
        detectors[i]->reset();
    }
}

void BPMonitor::printCSVHeader() const
{
    // Format: Time,Pressure,PPGSignal,Detector1,Detector2,...,DetectorN
    Serial.print("Time,Pressure,PPGSignal");
    for (int i = 0; i < detectorCount; i++)
    {
        Serial.print(",");
        Serial.print(detectors[i]->getName());
    }
    Serial.println();
}

void BPMonitor::printCSVRow(float pressure, int ppgSignal)
{
    // Format: Time,Pressure,PPGSignal,Detector1,Detector2,...,DetectorN
    Serial.print(millis());
    Serial.print(",");
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.print(ppgSignal);

    // Print detectors if in measuring or complete state
    bool shouldDetect = (state == MEASURING || state == COMPLETE);

    // Run all detectors and print results
    for (int i = 0; i < detectorCount; i++)
    {
        Serial.print(",");
        if (shouldDetect)
        {
            if (detectors[i]->getSystolic() == 0)
                detectors[i]->detect(ppgSignal, pressure);

            Serial.print(detectors[i]->getSystolic());
        }
        else
        {
            Serial.print("0"); // Not detecting yet
        }
    }
    Serial.println();
}

void BPMonitor::update(float pressure, int ppgSignal, Display &display)
{
    // Track max pressure
    if (pressure > maxPressure)
    {
        maxPressure = pressure;
    }

    switch (state)
    {
    case IDLE:
        display.print("Waiting...");
        if (pressure > BP_MIN_PRESSURE)
        {
            state = INFLATING;
        }
        break;

    case INFLATING:
        display.print("Inflating cuff...", "",
                      "Pressure: " + String((int)pressure) + " mmHg");

        if (pressure >= BP_START_PRESSURE)
        {
            state = MEASURING;
            startTime = millis();
        }
        else if (pressure < BP_MIN_PRESSURE && maxPressure > BP_START_PRESSURE)
        {
            reset();
        }
        break;

    case MEASURING:
        display.print("Deflating cuff...", "",
                      "Pressure: " + String((int)pressure) + " mmHg");

        // Print CSV data every sample
        if (pressure < (maxPressure - PRESSURE_DROP_THRESHOLD))
        {
            // Check if we got first systolic detection
            if (systolic == 0)
            {
                for (int i = 0; i < detectorCount; i++)
                {
                    // Check detectors for first count, we can put other logic here, empty for now
                    systolic = 1;
                }
            }
        }

        // Timeout
        if (millis() - startTime > 90000)
        {
            Serial.println("# Measurement timeout");
            state = COMPLETE;
        }

        // End when pressure drops low
        if (pressure < 10)
        {
            state = COMPLETE;
        }
        break;

    case COMPLETE:
        display.print("Measurement", "Complete");
        break;
    }
}

float BPMonitor::getSystolic() const
{
    return systolic;
}

BPState BPMonitor::getState() const
{
    return state;
}