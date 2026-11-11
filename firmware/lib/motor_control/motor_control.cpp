#include "motor_control.h"

MotorController::MotorController(uint8_t rpwm_pin, uint8_t lpwm_pin, uint8_t enable_pin)
    : rpwmPin(rpwm_pin), lpwmPin(lpwm_pin), enablePin(enable_pin),
      maxDuty(255), slewPerSecond(0.0f), targetFraction(0.0f),
      outputFraction(0.0f), coasting(false), lastUpdateUs(0),
      rpwmDuty(0), lpwmDuty(0), enableLevel(-1) {}

void MotorController::begin(uint32_t pwmFrequencyHz, uint8_t pwmResolutionBits,
                            float slewPercentPerSecond) {
    // bridge off while the PWM pins are being attached (the pull-downs hold
    // RPWM/LPWM low until then)
    pinMode(enablePin, OUTPUT);
    writeEnable(LOW);

    maxDuty = (1UL << pwmResolutionBits) - 1;
    slewPerSecond = slewPercentPerSecond > 0.0f ? slewPercentPerSecond / 100.0f : 0.0f;

    // same frequency, so ESP32PWM puts both on one LEDC timer
    rpwm.attachPin(rpwmPin, pwmFrequencyHz, pwmResolutionBits);
    lpwm.attachPin(lpwmPin, pwmFrequencyHz, pwmResolutionBits);
    rpwm.write(0);
    lpwm.write(0);
    rpwmDuty = 0;
    lpwmDuty = 0;

    lastUpdateUs = micros();
    brake();  // stopped, and held stopped, until something asks otherwise
}

void MotorController::setTarget(float fraction) {
    if (isnan(fraction)) fraction = 0.0f;
    targetFraction = constrain(fraction, -1.0f, 1.0f);
}

void MotorController::update() {
    unsigned long now = micros();
    float dt = (now - lastUpdateUs) * 1e-6f;  // unsigned, so rollover safe
    lastUpdateUs = now;

    float next = targetFraction;
    if (slewPerSecond > 0.0f) {
        float maxStep = slewPerSecond * dt;
        float delta = targetFraction - outputFraction;
        if (delta > maxStep) delta = maxStep;
        else if (delta < -maxStep) delta = -maxStep;
        next = outputFraction + delta;
    }
    // A change of direction never jumps across zero, even with the ramp off:
    // land on 0 (brake) for this update, the other side starts on the next.
    if ((outputFraction > 0.0f && next < 0.0f) ||
        (outputFraction < 0.0f && next > 0.0f)) {
        next = 0.0f;
    }

    outputFraction = next;
    apply(outputFraction);
}

void MotorController::brake() {
    targetFraction = 0.0f;
    outputFraction = 0.0f;
    coasting = false;
    apply(0.0f);
}

void MotorController::coast() {
    targetFraction = 0.0f;
    outputFraction = 0.0f;
    coasting = true;
    apply(0.0f);
}

int MotorController::outputPercent() const {
    return (int)lroundf(outputFraction * 100.0f);
}

int MotorController::direction() const {
    if (outputFraction > 0.0f) return 1;
    if (outputFraction < 0.0f) return -1;
    return 0;
}

// The one place the bridge pins change. The order inside each branch is the
// point: the side that must be off goes to 0 before EN and the driving side
// are touched, so RPWM and LPWM are never high together.
void MotorController::apply(float fraction) {
    uint32_t duty = (uint32_t)lroundf(fabsf(fraction) * (float)maxDuty);
    if (duty > maxDuty) duty = maxDuty;

    if (duty == 0) {
        writeRpwm(0);
        writeLpwm(0);
        writeEnable(coasting ? LOW : HIGH);
        return;
    }

    coasting = false;
    if (fraction > 0.0f) {
        writeLpwm(0);
        writeEnable(HIGH);
        writeRpwm(duty);
    } else {
        writeRpwm(0);
        writeEnable(HIGH);
        writeLpwm(duty);
    }
}

void MotorController::writeRpwm(uint32_t duty) {
    if (duty == rpwmDuty) return;
    rpwm.write(duty);
    rpwmDuty = duty;
}

void MotorController::writeLpwm(uint32_t duty) {
    if (duty == lpwmDuty) return;
    lpwm.write(duty);
    lpwmDuty = duty;
}

void MotorController::writeEnable(int level) {
    if (level == enableLevel) return;
    digitalWrite(enablePin, level);
    enableLevel = level;
}
