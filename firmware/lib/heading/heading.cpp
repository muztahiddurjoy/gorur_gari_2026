#include "heading.h"
#include "debug_serial.h"


Heading::Heading(): bno(nullptr), heading(0.0f){}

bool Heading::begin() {
    bno = new Adafruit_BNO055(55, BNO055_ADDRESS);
    if (!bno->begin()) {
        DEBUG_SERIAL.println("BNO055 NOT FOUND!");
        return false;
    }
    delay(1000);
    // bno->setExtCrystalUse(true);
    DEBUG_SERIAL.println("BNO055 Ready");
    return true;
}

void Heading::update() {
   if(!bno) return;
   sensors_event_t event;
    bno->getEvent(&event);
    heading = event.orientation.x;
}

void Heading::setHeading(float new_heading) {
    heading = new_heading;
}

void Heading::resetHeading() {
    heading = 0.0f;
}

float Heading::getHeading() {
    return heading;
}