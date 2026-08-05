#include "status_led.h"

StatusLed::StatusLed(int led_pin, uint8_t led_brightness){
    pin = led_pin;
    brightness = led_brightness;
    // -1 is a colour scale() can never produce, so the first set() always writes
    lastR = -1;
    lastG = -1;
    lastB = -1;
}

void StatusLed::begin(){
    // neopixelWrite() sets the RMT channel up on its own the first time it runs,
    // so there is no hardware to prepare here. Just make sure the next set()
    // is treated as a change even if the led kept its colour across a reset.
    lastR = -1;
    lastG = -1;
    lastB = -1;
}

int StatusLed::scale(uint8_t value){
    return ((int)value * (int)brightness) / 255;
}

void StatusLed::set(uint8_t r, uint8_t g, uint8_t b){
    int sr = scale(r);
    int sg = scale(g);
    int sb = scale(b);

    if(sr == lastR && sg == lastG && sb == lastB) return;

    lastR = sr;
    lastG = sg;
    lastB = sb;
    neopixelWrite(pin, sr, sg, sb);
}

void StatusLed::red(){
    set(255, 0, 0);
}

void StatusLed::green(){
    set(0, 255, 0);
}

void StatusLed::off(){
    set(0, 0, 0);
}
