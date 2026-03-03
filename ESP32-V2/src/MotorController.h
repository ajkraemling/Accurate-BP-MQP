#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
// Pump (Motor A)
#define MOTOR_IN1_PIN    25
#define MOTOR_IN2_PIN    26

// Valves
#define SLOW_SOLENOID_IN3_PIN 14
#define SLOW_SOLENOID_IN4_PIN 32

#define FAST_SOLENOID_CONTROL 2

// Button
#define BUTTON_PIN 13

#define MOTOR_PWM_FREQ 1000
#define MOTOR_PWM_CHANNEL 0
#define MOTOR_PWM_RES 8

#define SOLENOID_PWM_FREQ 1000
#define SOLENOID_PWM_CHANNEL 1
#define SOLENOID_PWM_RES 8

#define MOTOR_SPEED_MEDIUM 200
#define DEFLATE_RATE_SLOW 60
#define DEFLATE_RATE_MEDIUM 100
#define DEFLATE_RATE_FAST 140

class MotorController {
public:
    MotorController();
    void begin();

    void startInflation();
    void stopInflation();

    void startDeflation();
    void stopDeflation();

    void emergencyStop();

    bool isInflating() const { return inflating; }
    bool isDeflating() const { return deflating; }
    
    void openSlowSolenoid();
    void openFastSolenoid();
    void closeSlowSolenoid();
    void closeFastSolenoid();

private:
    bool inflating;
    bool deflating;

    void setMotorSpeed();
    void setMotorDirection();
};

#endif
