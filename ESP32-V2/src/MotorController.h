#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
// Pump (Motor A)
#define MOTOR_IN1_PIN    25
#define MOTOR_IN2_PIN    26
#define MOTOR_ENA_PIN    27

// Valve (Motor B)
#define SOLENOID_IN3_PIN 14
#define SOLENOID_IN4_PIN 32
#define SOLENOID_ENB_PIN 33

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

    void startInflation(int speed = 255);
    void stopInflation();

    void startDeflation(int rate = 80);
    void stopDeflation();

    void emergencyStop();

    bool isInflating() const { return inflating; }
    bool isDeflating() const { return deflating; }

private:
    bool inflating;
    bool deflating;
    int motorSpeed;
    int valveOpening;

    void setMotorSpeed(int speed);
    void setMotorDirection(bool forward);
    void setSolenoidOpening(int opening);
};

#endif
