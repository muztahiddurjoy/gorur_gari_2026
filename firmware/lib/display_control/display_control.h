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
        void begin();
        void clear();
        void displayText(const String& text, int x, int y, int textSize = 1);
        void display();
};

#endif