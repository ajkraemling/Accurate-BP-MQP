#include "output.h"
#include "config.h"

static unsigned long lastDisplayUpdate = 0;

void printInitializationHeader()
{
    Serial.println("\n\n===========================================================================================================");
    Serial.println("********************************************* Initialization **********************************************");
    Serial.println("===========================================================================================================");
}

void printOutputHeader()
{
    if (STREAM_MODE)
    {
        return; // No header in stream mode
    }

    if (DEBUG_MODE)
    {
        Serial.println("=== DEBUG MODE ===");
        Serial.println("========================================================================");
        Serial.println(" Heartbeat? | Raw PPG Signal |  Pulse Signal  |  Pressure (mmHg)");
        Serial.println("========================================================================");
    }
    else
    {
        Serial.println("=== HEARTBEAT LOGGING MODE ===");
    }
}

void outputData(bool heartbeatOccurred, bool beatDetected, int rawSignal,
                int filteredSignal, float pressure, unsigned long timestamp)
{
    if (STREAM_MODE) // For plotting
    {
        Serial.print(filteredSignal);
        Serial.print(",");
        Serial.println(pressure, 2);
    }
    else if (DEBUG_MODE) // For debugging, prints all data
    {
        if (timestamp - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL_MS)
        {
            lastDisplayUpdate = timestamp;

            String heartbeatStr = beatDetected ? "   YES    " : "    NO    ";

            String signalStr = String(filteredSignal);
            while (signalStr.length() < 4)
                signalStr = " " + signalStr;

            int pressureInt = round(pressure);
            String pressureStr = String(pressureInt);
            while (pressureStr.length() < 4)
                pressureStr = " " + pressureStr;

            Serial.println(heartbeatStr + " |     " + rawSignal + "       |        " +
                           signalStr + "       |        " + pressureStr);
        }
    }
    else // Prints pressure when heartbeat occurs
    {
        // Heartbeat logging mode
        if (heartbeatOccurred)
        {
            int pressureInt = round(pressure);
            Serial.println(String(timestamp) + "ms: Pressure at heartbeat: " +
                           String(pressureInt) + " mmHg");
        }
    }
}