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
        bool begin();
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
        bool ready;
};

#endif
