#ifndef CONFIG_H
#define CONFIG_H

// Pin Configuration
#define PULSESENSOR_OUT 15 // D15 on ESP32
#define RESET_PIN -1
#define EOC_PIN -1

// LCD Display Constants
#define LCD_COL 20
#define LCD_ROW 4

// Timing Constants
#define SAMPLE_DELAY_MS 50
#define BEAT_WINDOW_MS 250
#define THRESHOLD_UPDATE_INTERVAL_MS 5000
#define QUALITY_CHECK_INTERVAL_MS 2000
#define DISPLAY_UPDATE_INTERVAL_MS 20
#define CALIBRATION_DURATION_MS 5000
#define CALIBRATION_COUNTDOWN_SEC 5

// Blood Pressure Measurement
#define SYSTOLIC_START_PRESSURE 180     // Pressure to start looking for systolic
#define SYSTOLIC_MIN_PRESSURE 80        // Minimum valid systolic pressure
#define DIASTOLIC_MIN_PRESSURE 40       // Minimum valid diastolic pressure
#define PRESSURE_DROP_THRESHOLD 10      // Pressure must drop this much to start measurement
#define BP_MEASUREMENT_SENSITIVITY 0.25 // More sensitive thresholds during BP measurement (25% of range instead of 40-46%)
#define BP_BASELINE_WINDOW 40           // Number of samples for rolling baseline (2 seconds at 20Hz sampling)
#define BP_THRESHOLD_MULTIPLIER 2.5     // Standard deviations above baseline to detect pulse
#define MIN_DEVIATION_FROM_FLAT 50      // Minimum deviation to read pulse in case of flat line with no deviation

// Filter Sizes
#define PULSE_FILTER_SIZE 3
#define PRESSURE_FILTER_SIZE 4

// Threshold Ratios
#define UPPER_THRESHOLD_RATIO 0.46
#define LOWER_THRESHOLD_RATIO 0.40
#define ADAPTIVE_ALPHA 0.1 // 10% new, 90% old

// Conversion Constants
#define HPA_TO_MMHG 0.75006157584566

// Signal Quality Thresholds
#define MIN_SIGNAL_RANGE 20
#define MAX_SIGNAL_RANGE 3800
#define MIN_VALID_BEATS 3
#define MIN_HEART_RATE_BPM 40
#define MAX_HEART_RATE_BPM 180
#define STABILITY_THRESHOLD 0.6

// Output Mode Configuration
extern bool DEBUG_MODE;
extern bool STREAM_MODE;
extern bool DISPLAY_MODE;

#endif