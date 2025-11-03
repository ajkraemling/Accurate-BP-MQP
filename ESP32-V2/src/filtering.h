#ifndef FILTERING_H
#define FILTERING_H

#include "config.h"

typedef struct
{
    int buffer[PULSE_FILTER_SIZE];
    int index;
    int total;
} PulseFilterState;

typedef struct
{
    float buffer[PRESSURE_FILTER_SIZE];
    int index;
    float total;
    bool initialized;
    int initCounter;
} PressureFilterState;

// Initialize filter arrays
void initializeFilters(PulseFilterState *pulseFilter, PressureFilterState *pressureFilter);

// Apply moving average filter to pulse signal
int applyPulseFilter(PulseFilterState *filter, int newValue);

// Apply moving average filter to pressure signal
float applyPressureFilter(PressureFilterState *filter, float newValue);

// Check if pressure filter is initialized
bool isPressureInitialized(PressureFilterState *filter);

#endif