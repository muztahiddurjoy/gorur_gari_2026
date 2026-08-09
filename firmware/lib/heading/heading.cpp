#include "heading.h"
#include <Wire.h>
#include "debug_serial.h"

// register 0x00 holds the chip id, which is always 0xA0 on a BNO055
static const uint8_t BNO055_CHIP_ID_REG = 0x00;
static const uint8_t BNO055_CHIP_ID = 0xA0;

Heading::Heading(): bno(nullptr), heading(0.0f), ready(false){}

Heading::~Heading() {
    delete bno;
}

bool Heading::begin() {
    ready = false;
    // Adafruit_BNO055 news an Adafruit_I2CDevice in its constructor and has no
    // destructor to free it, so build the sensor object exactly once and let
    // retries re-run begin() on it - recreating it every attempt would leak a
    // little heap on every retry. begin() itself allocates nothing.
    if (!bno) {
        bno = new Adafruit_BNO055(55, BNO055_ADDRESS);
    }
    if (!bno->begin()) {
        DEBUG_SERIAL.println("BNO055 NOT FOUND!");
        return false;
    }
    delay(1000);
    // bno->setExtCrystalUse(true);
    DEBUG_SERIAL.println("BNO055 Ready");
    ready = true;
    return true;
}

bool Heading::isResponding() {
    Wire.beginTransmission(BNO055_ADDRESS);
    Wire.write(BNO055_CHIP_ID_REG);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)BNO055_ADDRESS, (uint8_t)1) != 1) return false;
    return Wire.read() == BNO055_CHIP_ID;
}

void Heading::markLost() {
    ready = false;
}

bool Heading::update() {
   if(!ready || !bno) return false;
   sensors_event_t event;
    bno->getEvent(&event);
    heading = event.orientation.x;
    return true;
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
