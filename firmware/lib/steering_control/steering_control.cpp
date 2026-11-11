#include "steering_control.h"

namespace {
// wire steering angle range, see mcu_bridge.py (steering_min/max/center_deg)
const int WIRE_ANGLE_MIN = 0;
const int WIRE_ANGLE_CENTER = 90;
const int WIRE_ANGLE_MAX = 180;
}  // namespace

SteeringController::SteeringController(uint8_t pin)
    : servoPin(pin), minPulseUs(0), centerPulseUs(0), maxPulseUs(0),
      lowUs(0), highUs(0), steeringAngle(WIRE_ANGLE_CENTER), pulseUs(0) {}

bool SteeringController::begin(int periodHz, int minUs, int centerUs, int maxUs) {
    // ESP32PWM::allocateTimer() is called once, at the top of setup()
    lowUs = min(minUs, maxUs);
    highUs = max(minUs, maxUs);
    minPulseUs = minUs;
    maxPulseUs = maxUs;
    centerPulseUs = constrain(centerUs, lowUs, highUs);

    servo.setPeriodHertz(periodHz);
    // ESP32Servo clamps every write to [lowUs, highUs] from here on. Its
    // attach() returns the channel number, which is not a success flag (an
    // MCPWM servo has none), so ask attached() instead.
    servo.attach(servoPin, lowUs, highUs);
    center();
    return servo.attached();
}

int SteeringController::angleToPulse(int angleDeg) const {
    angleDeg = constrain(angleDeg, WIRE_ANGLE_MIN, WIRE_ANGLE_MAX);
    long us;
    if (angleDeg <= WIRE_ANGLE_CENTER) {
        us = map(angleDeg, WIRE_ANGLE_MIN, WIRE_ANGLE_CENTER, minPulseUs, centerPulseUs);
    } else {
        us = map(angleDeg, WIRE_ANGLE_CENTER, WIRE_ANGLE_MAX, centerPulseUs, maxPulseUs);
    }
    return constrain((int)us, lowUs, highUs);
}

void SteeringController::to(int angleDeg) {
    steeringAngle = (uint8_t)constrain(angleDeg, WIRE_ANGLE_MIN, WIRE_ANGLE_MAX);
    int us = angleToPulse(steeringAngle);
    if (us == pulseUs) return;  // called every command; only touch the PWM on a change
    pulseUs = us;
    servo.writeMicroseconds(pulseUs);
}

void SteeringController::center() {
    to(WIRE_ANGLE_CENTER);
}
