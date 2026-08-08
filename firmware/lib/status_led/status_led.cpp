#include "status_led.h"

// Initialize the Adafruit_NeoPixel object in the constructor's initializer list
StatusLed::StatusLed(uint8_t pin, uint8_t count) 
    : ledPin(pin), numPixels(count), strip(count, pin, NEO_GRB + NEO_KHZ800) {}

void StatusLed::begin() {
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}

void StatusLed::setColor(uint8_t r, uint8_t g, uint8_t b) {
    // Assuming a single status LED, we address pixel index 0
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}

void StatusLed::setBrightness(uint8_t brightness) {
    strip.setBrightness(brightness);
    strip.show();
}

void StatusLed::turnOff() {
    strip.clear();
    strip.show();
}