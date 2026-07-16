#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <ESP32PWM.h>

// drives a single dc motor through an H-bridge with one pwm pin,
// two direction pins and a shared standby pin (TB6612FNG style).
class MotorController {
private:
    ESP32PWM throttle;
    int pwmPin;
    int inAPin;
    int inBPin;
    int standbyPin;
    int motorSpeed; // signed, sign is the direction
    int maxSpeed;
    void hold(int inA, int inB, int duty);
public:
    MotorController(int pwm_pin, int in_a_pin, int in_b_pin, int standby_pin);
    void begin(int frequency, int resolution);
    void to(int speed); // -maxSpeed..maxSpeed, 0 brakes
    void forward(int speed);
    void reverse(int speed);
    void brake();
    void coast();
    void standby(bool sleeping);
    int speed();
    int direction();
};

#endif // MOTOR_CONTROL_H
