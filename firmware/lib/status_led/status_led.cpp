#include "status_led.h"

StatusLed::StatusLed(uint8_t pin)
    : ledPin(pin), state(false) {}

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

void StatusLed::set(bool lit) {
    state = lit;
    digitalWrite(ledPin, lit ? HIGH : LOW);
}

void StatusLed::toggle() {
    set(!state);
}
