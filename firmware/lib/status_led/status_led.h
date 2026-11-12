#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

// A plain single colour LED, not the devkit's onboard WS2812.
// Wiring is active high: GPIO -> resistor (220-470R) -> LED anode, cathode to GND.
//
// Three things can want the LED, in this order of priority:
//   1. the start countdown: pressing the start button blinks it for exactly
//      as long as the ROS2 side's start delay, so the car shows when it is
//      about to move (blink()).
//   2. an alert pattern, for as long as one is set (setAlert()): the IMU
//      calibrating ("keep still") or faulted.
//   3. the level set() asked for: lit once the ROS2 bridge announces itself
//      over serial, and whatever mcu_bridge's /status_led topics ask after
//      that. While 1 or 2 owns the LED, set() only records the level, and the
//      LED returns to it when they end.
class StatusLed {
private:
    uint8_t ledPin;
    bool state;      // what the pin is actually at
    bool baseLevel;  // what set() last asked for

    // --- start countdown, driven by update(), never blocks ---
    uint8_t pulsesLeft;          // full on+off cycles still to run, 0 = idle
    unsigned long pulsePeriodMs; // one on+off cycle
    unsigned long phaseStartMs;  // when the current on (or off) phase began

    // --- alert pattern: lit for alertOnMs of every alertPeriodMs ---
    unsigned long alertPeriodMs; // 0 = no alert
    unsigned long alertOnMs;
    unsigned long alertStartMs;

    void write(bool lit);

public:
    explicit StatusLed(uint8_t pin);

    // configures the pin and starts with the LED off, so "lit" always means
    // the link came up after this boot
    void begin();

    void on();
    void off();
    void set(bool lit);
    void toggle();
    bool isOn() const { return state; }

    // Blink `times` full cycles, one cycle per `periodMs` (half lit, half dark),
    // then hand the LED back (alert pattern or set() level). Total runtime is
    // times * periodMs, so 5 x 1000 ms fills a 5 second countdown exactly.
    // Calling it again restarts the sequence.
    void blink(uint8_t times, unsigned long periodMs);
    bool isBlinking() const { return pulsesLeft > 0; }

    // Repeat "lit for onMs, dark for the rest of periodMs" until cleared with
    // setAlert(0, 0). Setting the same pattern again does not restart it, so
    // this can be called every loop().
    void setAlert(unsigned long periodMs, unsigned long onMs);
    bool hasAlert() const { return alertPeriodMs > 0; }

    // call every loop() - this is what actually advances blinks and patterns
    void update();
};

#endif
