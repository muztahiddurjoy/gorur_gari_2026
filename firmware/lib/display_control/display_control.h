#ifndef DISPLAY_CONTROL_H
#define DISPLAY_CONTROL_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1


class DisplayController{
    public:
        DisplayController();
        ~DisplayController();

        // safe to call over and over. on a cold boot the panel can still be
        // powering up when we get here, and one failed attempt used to leave the
        // screen dark until the next reboot, so the caller keeps retrying.
        // i2cClockHz: the bus clock the panel driver sets around each frame
        // (Adafruit_SSD1306 switches the clock for every transfer, so it has
        // to be told the bus speed everything else on the bus uses). A new
        // value rebuilds the driver object on the next begin().
        bool begin(uint32_t i2cClockHz);
        bool isReady() const { return ready; }

        // does the panel still ACK its address
        bool isResponding();

        // drops the panel back to "not ready" so the caller re-runs begin()
        void markLost();

        void clear();
        void displayText(const String& text, int x, int y, int textSize = 1);
        void display();
        void updateScreen();
    private:
        Adafruit_SSD1306* oled;
        uint32_t oledClockHz;
        bool ready;
};

#endif
