#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>

class SteeringController {
private:
    Servo steer;
    int servoPin;
    int servoCorrection = 23; // Correction factor for servo angle
    int steeringAngle;
public:
    SteeringController(int pin);
    void begin(int frequency);
    void to(int angle);
    uint8_t getAngle();
};

#endif // STEERING_CONTROL_H