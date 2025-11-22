#include "DisplayPresenter.h"
#include <stdio.h>
#include <string.h>

DisplayPresenter::DisplayPresenter(IDisplay* display)
    : display(display)
{
    memset(line1Buffer, 0, sizeof(line1Buffer));
    memset(line2Buffer, 0, sizeof(line2Buffer));
    memset(line3Buffer, 0, sizeof(line3Buffer));
    memset(line4Buffer, 0, sizeof(line4Buffer));
}

void DisplayPresenter::showStatus(const BPStatus& status)
{
    strncpy(line1Buffer, status.statusMessage, sizeof(line1Buffer) - 1);
    strncpy(line2Buffer, status.detailMessage, sizeof(line2Buffer) - 1);
    
    if (status.state == INFLATING || status.state == MEASURING)
    {
        snprintf(line3Buffer, sizeof(line3Buffer), "Pressure: %d mmHg", 
                 (int)status.currentPressure);
        display->showLines(line1Buffer, line2Buffer, line3Buffer);
    }
    else
    {
        display->showLines(line1Buffer, line2Buffer);
    }
}

void DisplayPresenter::showCountdown(int seconds)
{
    strncpy(line1Buffer, "CALIBRATION", sizeof(line1Buffer) - 1);
    strncpy(line2Buffer, "Place finger on", sizeof(line2Buffer) - 1);
    strncpy(line3Buffer, "sensor...", sizeof(line3Buffer) - 1);
    snprintf(line4Buffer, sizeof(line4Buffer), "%d...", seconds);
    
    display->showLines(line1Buffer, line2Buffer, line3Buffer, line4Buffer);
}

void DisplayPresenter::showCalibrating()
{
    display->showLines("Calibrating...");
}

void DisplayPresenter::showReady()
{
    display->showLines("Ready!");
}

void DisplayPresenter::showError(const char* message)
{
    display->showLines("ERROR:", message);
}