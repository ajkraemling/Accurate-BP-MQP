#ifndef DISPLAY_PRESENTER_H
#define DISPLAY_PRESENTER_H

#include "BPMonitor.h"

// Abstract display interface - no hardware dependency
class IDisplay
{
public:
    virtual ~IDisplay() {}
    virtual void showLines(const char* line1, const char* line2 = nullptr,
                          const char* line3 = nullptr, const char* line4 = nullptr) = 0;
    virtual void clear() = 0;
};

// Handles presentation logic - formats data for display
class DisplayPresenter
{
private:
    IDisplay* display;
    char line1Buffer[32];
    char line2Buffer[32];
    char line3Buffer[32];
    char line4Buffer[32];
    
public:
    DisplayPresenter(IDisplay* display);
    
    void showStatus(const BPStatus& status);
    void showCountdown(int seconds);
    void showCalibrating();
    void showReady();
    void showError(const char* message);
};

#endif