#include "filtering.h"

void initializeFilters(PulseFilterState *pulseFilter, PressureFilterState *pressureFilter)
{
    // Initialize pulse filter
    for (int i = 0; i < PULSE_FILTER_SIZE; i++)
    {
        pulseFilter->buffer[i] = 0;
    }
    pulseFilter->index = 0;
    pulseFilter->total = 0;

    // Initialize pressure filter
    for (int i = 0; i < PRESSURE_FILTER_SIZE; i++)
    {
        pressureFilter->buffer[i] = 0;
    }
    pressureFilter->index = 0;
    pressureFilter->total = 0;
    pressureFilter->initialized = false;
    pressureFilter->initCounter = 0;
}

int applyPulseFilter(PulseFilterState *filter, int newValue)
{
    filter->total = filter->total - filter->buffer[filter->index];
    filter->buffer[filter->index] = newValue;
    filter->total = filter->total + filter->buffer[filter->index];
    filter->index = (filter->index + 1) % PULSE_FILTER_SIZE;

    return filter->total / PULSE_FILTER_SIZE;
}

float applyPressureFilter(PressureFilterState *filter, float newValue)
{
    filter->total = filter->total - filter->buffer[filter->index];
    filter->buffer[filter->index] = newValue;
    filter->total = filter->total + filter->buffer[filter->index];
    filter->index = (filter->index + 1) % PRESSURE_FILTER_SIZE;

    // Check if filter is filled
    if (!filter->initialized)
    {
        filter->initCounter++;
        if (filter->initCounter > PRESSURE_FILTER_SIZE + 5)
        {
            filter->initialized = true;
        }
    }

    return filter->total / PRESSURE_FILTER_SIZE;
}

bool isPressureInitialized(PressureFilterState *filter)
{
    return filter->initialized;
}