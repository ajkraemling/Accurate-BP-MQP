// ── Display-Board ESP32 (ESP32-2432S032C) I2C Slave — with diagnostics ────────
//
// TFT  : ST7789 on HSPI  (MOSI=13, SCLK=14, CS=15, DC=2, BL=27)
// Comms: I2C slave on Wire (SDA=IO32, SCL=IO25)  address 0x42
//
// Wiring to master NodeMCU-32S (Wire, GPIO21/22):
//   Master GPIO 21 (SDA) ──── Display IO32 (SDA)
//   Master GPIO 22 (SCL) ──── Display IO25 (SCL)
//   Shared GND
//   (Pull-ups provided by MPRLS/LCD modules already on the master bus)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

// ── I2C slave config ──────────────────────────────────────────────────────────
#define I2C_SDA_PIN  32   // IO32
#define I2C_SCL_PIN  25   // IO25
#define SLAVE_ADDR   0x42

// ── Timeouts ─────────────────────────────────────────────────────────────────
#define NO_DATA_WARN_MS   5000   // serial warning if no data for this long
#define LOST_TIMEOUT_MS  10000   // switch LCD to LOST state after this long

// ── TFT ───────────────────────────────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();

// ── Connection state ─────────────────────────────────────────────────────────
enum ConnState { WAITING, CONNECTED, LOST };
ConnState connState = WAITING;

// ── Shared state (ISR writes, loop reads) ────────────────────────────────────
volatile float receivedPressure = 0.0f;
volatile bool  newData          = false;

uint32_t lastDataMs         = 0;   // millis() of last received packet
uint32_t packetsReceived    = 0;   // total packets since boot
uint32_t lastWatchdogPrint  = 0;   // throttle for serial watchdog messages
uint32_t lastWaitingLCDMs   = 0;   // throttle for WAITING lcd refresh

// ── I2C receive callback (ISR context — keep it fast) ────────────────────────
void onReceive(int numBytes)
{
    if (numBytes >= 4) {
        uint8_t buf[4];
        for (int i = 0; i < 4 && Wire.available(); i++) {
            buf[i] = Wire.read();
        }
        float val;
        memcpy(&val, buf, sizeof(float));
        receivedPressure = val;
        newData          = true;
    }
    while (Wire.available()) Wire.read();
}

// ── LCD drawing helpers ───────────────────────────────────────────────────────

// Small status bar at the very top of the screen (y=0..22)
void drawStatusBar(const char* label, uint16_t color)
{
    tft.fillRect(0, 0, 320, 22, TFT_NAVY);
    tft.setTextColor(color, TFT_NAVY);
    tft.setTextSize(2);
    tft.setCursor(6, 3);
    tft.print(label);
}

// WAITING state — shows elapsed time and prompts
void drawWaitingScreen(uint32_t elapsedSec)
{
    tft.fillScreen(TFT_BLACK);
    drawStatusBar("WAITING FOR MASTER", TFT_YELLOW);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 40);
    tft.print("No data received");

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 70);
    tft.print("Elapsed: ");
    tft.print(elapsedSec);
    tft.print(" s");

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 110);
    tft.print("Check wiring:");
    tft.setCursor(10, 125);
    tft.print("Master GPIO21 -> Display IO32 (SDA)");
    tft.setCursor(10, 140);
    tft.print("Master GPIO22 -> Display IO25 (SCL)");
    tft.setCursor(10, 155);
    tft.print("Shared GND required");
}

// CONNECTED state — big pressure value + small status bar
void drawPressureScreen(float pressure)
{
    tft.fillScreen(TFT_BLACK);
    drawStatusBar("CONNECTED  |  RECEIVING", TFT_GREEN);

    // Label
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(10, 60);
    tft.print("Pressure");

    // Value — large
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(6);
    tft.setCursor(10, 120);
    tft.print((int)pressure);

    // Unit
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(10, 220);
    tft.print("mmHg");

    // Packet counter (bottom-right, small)
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(240, 228);
    tft.print("pkts:");
    tft.print(packetsReceived);
}

// LOST state — data was flowing, now stopped
void drawLostScreen(uint32_t secSinceLast)
{
    tft.fillScreen(TFT_BLACK);
    drawStatusBar("SIGNAL LOST", TFT_RED);

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 40);
    tft.print("Data stream stopped");

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 68);
    tft.print("Last packet: ");
    tft.print(secSinceLast);
    tft.print(" s ago");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.print("Possible causes:");

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 125);
    tft.print("- Master restarted or crashed");
    tft.setCursor(10, 140);
    tft.print("- I2C bus floated (pull-ups missing?)");
    tft.setCursor(10, 155);
    tft.print("- Loose wire on SDA or SCL");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    // Backlight first
    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);

    Serial.begin(115200);
    Serial.println("[display] booting...");
    Serial.println("[display] TFT: HSPI MOSI=13 SCLK=14 CS=15 DC=2 BL=27");
    Serial.println("[display] I2C slave: SDA=IO32 SCL=IO25 addr=0x42");

    // TFT on HSPI (USE_HSPI_PORT in build flags)
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_RED);
    delay(300);
    tft.fillScreen(TFT_BLACK);
    Serial.println("[display] TFT OK (red flash complete)");

    drawWaitingScreen(0);

    // I2C slave on Wire, address 0x42
    Wire.begin(SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.onReceive(onReceive);
    Serial.println("[display] I2C slave registered at 0x42 (SDA=IO32, SCL=IO25) — waiting for master");

    lastDataMs       = millis();
    lastWaitingLCDMs = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop()
{
    uint32_t now = millis();

    // ── New data received ─────────────────────────────────────────────────────
    if (newData) {
        newData = false;
        float pressure = receivedPressure;
        packetsReceived++;
        lastDataMs = now;
        connState  = CONNECTED;

        Serial.print("[display] Received pressure: ");
        Serial.print(pressure, 1);
        Serial.println(" mmHg");
        Serial.println("[display] Updating display");

        drawPressureScreen(pressure);

        Serial.println("[display] Display refresh triggered — rendering complete");
    }

    // ── Watchdog: check for data timeout ─────────────────────────────────────
    uint32_t msSinceLast = now - lastDataMs;

    if (connState == WAITING) {
        // Never connected yet — refresh waiting screen every 5 s
        if (now - lastWaitingLCDMs >= 5000) {
            lastWaitingLCDMs = now;
            drawWaitingScreen(now / 1000);
        }
        // Serial warning every 5 s
        if (now >= 5000 && now - lastWatchdogPrint >= 5000) {
            lastWatchdogPrint = now;
            Serial.println("[display] No data received for 5+ seconds");
            Serial.println("[display] Possible issues:");
            Serial.println("[display]   - SDA/SCL swapped (try swap IO32/IO25)");
            Serial.println("[display]   - Missing common ground");
            Serial.println("[display]   - Wrong pins: need master GPIO21->IO32, GPIO22->IO25");
            Serial.println("[display]   - Master Wire.begin() not called");
            Serial.println("[display]   - Wrong slave address (need 0x42)");
        }
    } else if (connState == CONNECTED && msSinceLast >= LOST_TIMEOUT_MS) {
        // Was connected, now lost
        connState = LOST;
        drawLostScreen(msSinceLast / 1000);
        Serial.print("[display] Data stream lost (");
        Serial.print(msSinceLast / 1000);
        Serial.println("s since last packet)");
    } else if (connState == LOST && msSinceLast >= LOST_TIMEOUT_MS) {
        // Repeat serial warning every 5 s while lost
        if (now - lastWatchdogPrint >= 5000) {
            lastWatchdogPrint = now;
            Serial.print("[display] Still no data — ");
            Serial.print(msSinceLast / 1000);
            Serial.println("s since last packet");
            Serial.println("[display]   - Check SDA/SCL wiring and pull-ups");
        }
    }

    delay(10);
}
