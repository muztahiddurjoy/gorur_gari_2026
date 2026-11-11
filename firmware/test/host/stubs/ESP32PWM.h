// Host-side stand-in for ESP32Servo's ESP32PWM: records every duty write
// into host_pin_log so the motor tests can check what reached which pin, and
// in which order.
#pragma once

#include "Arduino.h"

class ESP32PWM {
public:
    void attachPin(uint8_t p, double, uint8_t) { pin = p; attachedState = true; }
    void write(uint32_t duty) { host_pin_log.push_back({"pwm", pin, duty}); }
    bool attached() const { return attachedState; }
    static void allocateTimer(int) {}

private:
    int pin = -1;
    bool attachedState = false;
};
