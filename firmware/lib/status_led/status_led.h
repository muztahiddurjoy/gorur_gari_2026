#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

// A plain single colour LED, not the devkit's onboard WS2812 on GPIO48.
// Wiring is active high: GPIO -> resistor (220-470R) -> LED anode, cathode to GND.
// It goes high once the ROS2 bridge announces itself over serial, so a glance at
// the board tells you whether anything is actually talking to the MCU.
//
// It doubles as the start countdown: pressing the start button kicks off a
// blink sequence that runs for exactly as long as the ROS2 side's start delay,
// so the car itself shows when it is about to move.
class StatusLed {
private:
    uint8_t ledPin;
    bool state;

    // --- blink sequence, driven by update(), never blocks ---
    uint8_t pulsesLeft;          // full on+off cycles still to run, 0 = idle
    unsigned long pulsePeriodMs; // one on+off cycle
    unsigned long phaseStartMs;  // when the current on (or off) phase began
    bool restoreState;           // level to go back to once the sequence ends

    void write(bool lit);

public:
    explicit StatusLed(uint8_t pin);

    // configures the pin and starts with the LED off, so "lit" always means
    // the link came up after this boot
    void begin();

    void on();
    void off();
    // While a blink sequence is running this only records where to return to -
    // the sequence owns the pin until it finishes, so a connect/disconnect
    // message arriving mid countdown cannot cut the blinking short.
    void set(bool lit);
    void toggle();
    bool isOn() const { return state; }

    // Blink `times` full cycles, one cycle per `periodMs` (half lit, half dark),
    // then return to the level the LED was at. Total runtime is
    // times * periodMs, so 3 x 1000 ms fills a 3 second countdown exactly.
    // Calling it again restarts the sequence.
    void blink(uint8_t times, unsigned long periodMs);
    bool isBlinking() const { return pulsesLeft > 0; }
    // call every loop() - this is what actually advances the sequence
    void update();
};

#endif
