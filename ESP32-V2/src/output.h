#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>

struct BPMeasurementData;

// Print initialization header
void printInitializationHeader();

// Print output mode header
void printOutputHeader();

// Output data based on current mode
void outputData(bool heartbeatOccurred, bool beatDetected, int rawSignal,
                int filteredSignal, float pressure, unsigned long timestamp, BPMeasurementData *bpData);

#endif