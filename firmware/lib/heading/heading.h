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
        bool begin();
        void update();
        float getHeading();
        void setHeading(float new_heading);
        void resetHeading();
    private:
        Adafruit_BNO055* bno;
        float heading;
};

#endif