#include "status_led.h"

StatusLed::StatusLed(uint8_t pin)
    : ledPin(pin), state(false), pulsesLeft(0), pulsePeriodMs(0),
      phaseStartMs(0), restoreState(false) {}

void StatusLed::begin() {
    pinMode(ledPin, OUTPUT);
    off();
}

void StatusLed::on() {
    set(true);
}

void StatusLed::off() {
    set(false);
}

void StatusLed::write(bool lit) {
    state = lit;
    digitalWrite(ledPin, lit ? HIGH : LOW);
}

void StatusLed::set(bool lit) {
    if (isBlinking()) {
        // the countdown owns the pin; remember where to land when it ends
        restoreState = lit;
        return;
    }
    write(lit);
}

void StatusLed::toggle() {
    set(!state);
}

void StatusLed::blink(uint8_t times, unsigned long periodMs) {
    if (times == 0 || periodMs == 0) return;
    // a restart mid sequence must not capture the blink's own output as the
    // level to return to
    if (!isBlinking()) {
        restoreState = state;
    }
    pulsesLeft = times;
    pulsePeriodMs = periodMs;
    phaseStartMs = millis();
    write(true);  // every cycle starts lit
}

void StatusLed::update() {
    if (!isBlinking()) return;

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
    if (pulsesLeft == 0) {
        write(restoreState);
        return;
    }
    phaseStartMs += pulsePeriodMs;  // no drift across cycles
    write(true);
}
