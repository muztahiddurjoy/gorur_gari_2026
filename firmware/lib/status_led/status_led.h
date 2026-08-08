#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

// A plain single colour LED, not the devkit's onboard WS2812 on GPIO48.
// Wiring is active high: GPIO -> resistor (220-470R) -> LED anode, cathode to GND.
// It goes high once the ROS2 bridge announces itself over serial, so a glance at
// the board tells you whether anything is actually talking to the MCU.
class StatusLed {
private:
    uint8_t ledPin;
    bool state;

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
};

#endif
