#include "status_led.h"

StatusLed::StatusLed(uint8_t pin)
    : ledPin(pin), state(false), baseLevel(false), pulsesLeft(0), pulsePeriodMs(0),
      phaseStartMs(0), alertPeriodMs(0), alertOnMs(0), alertStartMs(0) {}

void StatusLed::begin() {
    pinMode(ledPin, OUTPUT);
    baseLevel = false;
    state = true;  // force the first write
    write(false);
}

void StatusLed::on() {
    set(true);
}

void StatusLed::off() {
    set(false);
}

void StatusLed::write(bool lit) {
    if (lit == state) return;
    state = lit;
    digitalWrite(ledPin, lit ? HIGH : LOW);
}

void StatusLed::set(bool lit) {
    // remembered either way; shown at once only if nothing else owns the LED
    // (a connect/disconnect message mid countdown cannot cut it short)
    baseLevel = lit;
    if (!isBlinking() && !hasAlert()) {
        write(lit);
    }
}

void StatusLed::toggle() {
    set(!baseLevel);
}

void StatusLed::blink(uint8_t times, unsigned long periodMs) {
    if (times == 0 || periodMs == 0) return;
    pulsesLeft = times;
    pulsePeriodMs = periodMs;
    phaseStartMs = millis();
    write(true);  // every cycle starts lit
}

void StatusLed::setAlert(unsigned long periodMs, unsigned long onMs) {
    if (periodMs == alertPeriodMs && onMs == alertOnMs) return;
    alertPeriodMs = periodMs;
    alertOnMs = onMs;
    alertStartMs = millis();
}

void StatusLed::update() {
    if (isBlinking()) {
        unsigned long elapsed = millis() - phaseStartMs;
        unsigned long halfPeriod = pulsePeriodMs / 2;

        if (state) {
            // lit half over -> go dark for the rest of this cycle
            if (elapsed >= halfPeriod) {
                write(false);
            }
            return;
        }

        // dark half still running
        if (elapsed < pulsePeriodMs) return;

        // cycle done
        pulsesLeft--;
        if (pulsesLeft > 0) {
            phaseStartMs += pulsePeriodMs;  // no drift across cycles
            write(true);
            return;
        }
        // countdown over: fall through to whoever owns the LED next
    }

    if (hasAlert()) {
        // phase from elapsed time, so a slow pass never leaves it stuck
        unsigned long phase = (millis() - alertStartMs) % alertPeriodMs;
        write(phase < alertOnMs);
        return;
    }

    write(baseLevel);
}
