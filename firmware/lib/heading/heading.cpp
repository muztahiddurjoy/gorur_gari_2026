#include "heading.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



Adafruit_BNO055 bno(55, BNO055_ADDRESS);

Heading::Heading(){
    heading = 0.0f;
}

void Heading::begin() {
    if (!bno.begin()) {
        Serial.println("BNO055 NOT FOUND!");
    }

    delay(1000);
    // bno.setExtCrystalUse(true);
    Serial.println("BNO055 Ready");
}

void Heading::update() {
    sensors_event_t event;
    bno.getEvent(&event);
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