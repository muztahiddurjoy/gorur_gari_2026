#ifndef HEADING_H
#define HEADING_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "config.h"

class Heading{
    public:
        Heading();
        ~Heading();

        // safe to call over and over. the BNO055 needs 650 ms of power on reset
        // before it answers on I2C at all, and Adafruit's begin() gives up on the
        // first NACK, so a cold boot can easily lose the race. returns whether the
        // sensor is up, and the caller is expected to keep retrying until it is.
        bool begin();
        bool isReady() const { return ready; }

        // reads the chip id register, which is 0xA0 on a healthy BNO055. an
        // address ping is not enough here: the chip keeps ACKing its address after
        // a brownout while quietly serving zeroes, so check something with a known
        // value instead.
        bool isResponding();

        // drops the sensor back to "not ready" so the caller re-runs begin()
        void markLost();

        // false when the sensor is not up, so a dead IMU leaves the last known
        // heading alone instead of overwriting it with a zero
        bool update();

        float getHeading();
        void setHeading(float new_heading);
        void resetHeading();
    private:
        Adafruit_BNO055* bno;
        float heading;
        bool ready;
};

#endif
