#include "button_handler.h"

ButtonHandler::ButtonHandler(uint8_t pin, unsigned long delayTime) 
    : buttonPin(pin), debounceDelay(delayTime), lastDebounceTime(0), lastButtonState(HIGH), buttonState(HIGH) {}

void ButtonHandler::begin() {
    pinMode(buttonPin, INPUT_PULLUP);
}

bool ButtonHandler::isPressed() {
    bool reading = digitalRead(buttonPin);
    if (reading != lastButtonState) {
        lastDebounceTime = millis(); 
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;
            if (buttonState == LOW) {
                lastButtonState = reading;
                return true; 
            }
        }
    }

    lastButtonState = reading;
    return false;
}