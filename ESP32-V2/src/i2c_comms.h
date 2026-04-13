#ifndef I2C_COMMS_H
#define I2C_COMMS_H

// ─────────────────────────────────────────────────────────────────────────────
// Inter-ESP32 I2C Communication
//
// Master (nodemcu-32s) uses Wire1 (its second I2C bus) so Wire is free for the
// MPRLS pressure sensor and LCD.
//
// Slave  (display-board) uses Wire (its only needed I2C bus).
// ─────────────────────────────────────────────────────────────────────────────

// Wire1 pins on the master – choose GPIO pins not already in use
#define INTER_ESP_SDA  25
#define INTER_ESP_SCL  26

// I2C address the display-board slave responds on
#define SLAVE_I2C_ADDR 0x42

// ── State codes ──────────────────────────────────────────────────────────────
#define BP_STATE_IDLE     0x00  // waiting / ready
#define BP_STATE_INFLATE  0x01  // cuff inflating
#define BP_STATE_MEASURE  0x02  // measuring (deflating)
#define BP_STATE_COMPLETE 0x03  // results ready
#define BP_STATE_ERROR    0x04  // sensor error

// ── Packet sent master → slave ───────────────────────────────────────────────
// Keep fields word-aligned; packed to avoid padding surprises over the wire.
struct __attribute__((packed)) BPPacket {
    uint8_t  state;       // one of BP_STATE_* above
    int16_t  systolic;    // mmHg  (0 until COMPLETE)
    int16_t  diastolic;   // mmHg  (0 until COMPLETE)
    int16_t  map;         // mmHg  (mean arterial pressure, 0 until COMPLETE)
    int16_t  bpm;         // beats/min (0 until COMPLETE)
    int16_t  pressure;    // current cuff pressure in mmHg (live during measure)
};

#define BP_PACKET_SIZE  sizeof(BPPacket)    // 11 bytes

#endif // I2C_COMMS_H
