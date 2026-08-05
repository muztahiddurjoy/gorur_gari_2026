#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

// The single WS2812 soldered onto the devkit, used as a link indicator:
// red while nothing has the USB serial port open, green once the ROS2 side
// (mcu_bridge) is attached.
//
// set() remembers the last colour and skips redundant writes, because every
// write is a blocking RMT transaction and loop() runs far faster than the
// colour actually changes.
class StatusLed {
private:
    int pin;
    uint8_t brightness;
    int lastR;
    int lastG;
    int lastB;
    int scale(uint8_t value);
public:
    StatusLed(int led_pin, uint8_t led_brightness);
    void begin();
    void set(uint8_t r, uint8_t g, uint8_t b);
    void red();
    void green();
    void off();
};

#endif // STATUS_LED_H
