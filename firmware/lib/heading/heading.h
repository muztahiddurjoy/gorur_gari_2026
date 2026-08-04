#ifndef HEADING_H
#define HEADING_H

#include <Arduino.h>
#include "config.h"

class Heading{
    private:
        float heading;
    public:
        Heading();
        void begin();
        void update();
        float getHeading();
        void setHeading(float new_heading);
        void resetHeading();
};

#endif