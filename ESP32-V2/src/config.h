#ifndef CONFIG_H
#define CONFIG_H

// Pin Configuration
#define PPG_PIN 15
#define RESET_PIN -1
#define EOC_PIN -1

// LCD Configuration
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Timing
#define SAMPLE_RATE_MS 20.0f
#define CALIBRATION_TIME_MS 5000

// MAP
#define TREND_WINDOW_MS 200.0f
#define TREND_WINDOW ((int)(TREND_WINDOW_MS / SAMPLE_RATE_MS))

// BP Measurement Thresholds
#define BP_START_PRESSURE 140
#define BP_MIN_IDLE_PRESSURE 30
#define PRESSURE_DROP_THRESHOLD 15

// Heartbeat intervals
#define MIN_BEAT_INTERVALS_MS 300 // 200 bpm
#define MAX_BEAT_INTERVALS_MS 1500 // 40 bpm

// Conversion
#define HPA_TO_MMHG 0.75006157584566

// Ensemble Reporting
#define MAX_DETECTIONS 20
#define MAX_DETECTORS 155

#endif