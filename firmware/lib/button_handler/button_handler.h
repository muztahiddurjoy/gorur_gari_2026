#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

class ButtonHandler {
private:
    uint8_t buttonPin;
    unsigned long lastDebounceTime; 
    unsigned long debounceDelay;    
    bool lastButtonState;
    bool buttonState;

public:
    ButtonHandler(uint8_t pin, unsigned long delayTime = 50); // Default 50ms debounce
    void begin();
    bool isPressed();
    // debounced level, true for as long as the button is held down. isPressed()
    // only fires once on the press edge, so telemetry that samples slower than a
    // tap would miss it; this reports the state isPressed() last settled on.
    bool isDown() const;
};

#endif