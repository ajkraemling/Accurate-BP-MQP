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
#define SAMPLE_RATE_MS 20
#define CALIBRATION_TIME_MS 5000

// BP Measurement Thresholds
#define BP_START_PRESSURE 180
#define BP_MIN_PRESSURE 80
#define PRESSURE_DROP_THRESHOLD 10

// Conversion
#define HPA_TO_MMHG 0.75006157584566

#endif