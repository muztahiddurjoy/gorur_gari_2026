#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <ESP32PWM.h>

// One DC motor on a BTS7960 (IBT-2) dual half-bridge: RPWM drives forward,
// LPWM drives reverse, and R_EN + L_EN are tied together on one enable pin.
//
//   forward  LPWM = 0 first, then EN = HIGH, then RPWM = duty
//   reverse  RPWM = 0 first, then EN = HIGH, then LPWM = duty
//   brake    RPWM = LPWM = 0, EN = HIGH  (both low sides on, windings shorted)
//   coast    RPWM = LPWM = 0, EN = LOW   (bridge off, motor freewheels)
//
// RPWM and LPWM are never both non-zero, and the output is slew limited:
// setTarget() only says where to go, update() moves the output towards it at
// no more than the configured rate, and a change of direction always stops at
// zero (brake) for at least one update before the other side starts driving.
//
// brake() and coast() are the exceptions: they act on the spot, with no ramp,
// because they are what the safety paths call.
//
// The board has 10k pull-downs on RPWM, LPWM and EN, so the bridge is off from
// power-up until begin() runs, and begin() leaves it braked.
class MotorController {
public:
    MotorController(uint8_t rpwmPin, uint8_t lpwmPin, uint8_t enablePin);

    // Attaches both PWM pins (two ESP32PWM / LEDC channels, so call
    // ESP32PWM::allocateTimer() first) and leaves the motor braked.
    // slewPercentPerSecond: how fast the output may change, in percent of full
    // duty per second (500 = 0 to full in 200 ms). <= 0 disables the ramp.
    void begin(uint32_t pwmFrequencyHz, uint8_t pwmResolutionBits,
               float slewPercentPerSecond);

    // -1.0 (full reverse) .. 0 .. +1.0 (full forward). Out of range values are
    // clamped. Takes effect through update().
    void setTarget(float fraction);

    // Call every loop(). Advances the ramp by the time since the last call and
    // writes the bridge if the output changed. Never blocks.
    void update();

    // Immediate stop with the windings shorted, no ramp. Target becomes 0.
    void brake();
    // Immediate stop with the bridge disabled, no ramp. Target becomes 0.
    // The bridge is re-enabled by the next non-zero output.
    void coast();

    float target() const { return targetFraction; }
    // what is on the bridge right now, signed, -1..1
    float output() const { return outputFraction; }
    // the same, as a rounded signed percentage, for the OLED / status pixel
    int outputPercent() const;
    int direction() const;  // +1 forward, -1 reverse, 0 stopped

private:
    ESP32PWM rpwm;
    ESP32PWM lpwm;
    uint8_t rpwmPin;
    uint8_t lpwmPin;
    uint8_t enablePin;

    uint32_t maxDuty;           // 2^bits - 1, the core maps it to fully on
    float slewPerSecond;        // fraction of full duty per second, 0 = no ramp
    float targetFraction;
    float outputFraction;
    bool coasting;              // EN held low until something drives again
    unsigned long lastUpdateUs;

    // last values actually written, so update() can run every loop without
    // rewriting the same registers thousands of times a second
    uint32_t rpwmDuty;
    uint32_t lpwmDuty;
    int enableLevel;            // -1 = never written

    void apply(float fraction);
    void writeRpwm(uint32_t duty);
    void writeLpwm(uint32_t duty);
    void writeEnable(int level);
};

#endif // MOTOR_CONTROL_H
