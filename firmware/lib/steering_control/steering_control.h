#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>

// The steering servo, driven in microseconds between three calibrated pulse
// widths so the linkage can never be pushed into its end stops.
//
// The wire carries a steering angle 0..180 with 90 = straight (mcu_bridge's
// steering_center_deg). to() maps it piecewise-linearly:
//
//     angle   0 .. 90  ->  minUs .. centerUs
//     angle  90 .. 180 ->  centerUs .. maxUs
//
// so the whole wire range lands inside [minUs, maxUs], the centre is trimmed
// independently of the travel, and an asymmetric linkage just gets different
// distances either side. ESP32Servo is attached with the same limits, so even
// a direct writeMicroseconds() cannot leave them.
class SteeringController {
public:
    explicit SteeringController(uint8_t pin);

    // Attaches the servo at periodHz and drives it to the centre pulse.
    // minUs/maxUs may be given in either order (swap them if the wheels turn
    // the wrong way); centerUs is clamped between them.
    bool begin(int periodHz, int minUs, int centerUs, int maxUs);

    // wire angle 0..180 (clamped), 90 = straight
    void to(int angleDeg);
    void center();

    uint8_t getAngle() const { return steeringAngle; }  // last commanded wire angle
    int getMicroseconds() const { return pulseUs; }     // pulse actually sent

private:
    Servo servo;
    uint8_t servoPin;
    int minPulseUs;     // pulse for wire angle 0
    int centerPulseUs;  // pulse for wire angle 90
    int maxPulseUs;     // pulse for wire angle 180
    int lowUs, highUs;  // the same limits, sorted
    uint8_t steeringAngle;
    int pulseUs;

    int angleToPulse(int angleDeg) const;
};

#endif // STEERING_CONTROL_H
