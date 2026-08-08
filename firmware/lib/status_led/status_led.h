#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class StatusLed {
private:
    uint8_t ledPin;
    uint8_t numPixels;
    Adafruit_NeoPixel strip; // Encapsulate the strip object within the class

public:
    // Constructor defaults to standard ESP32-S3 DevKitC parameters (pin 48, 1 pixel)
    StatusLed(uint8_t pin = 48, uint8_t count = 1);
    
    void begin();
    
    // Helper methods to make controlling the LED easier in your main code
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t brightness);
    void turnOff();
};

#endif