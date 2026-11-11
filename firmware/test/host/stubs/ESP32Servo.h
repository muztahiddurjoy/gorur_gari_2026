// Host-side stand-in for ESP32Servo's Servo: remembers the attach limits and
// every pulse written, so the steering tests can check the limits hold.
#pragma once

#include <algorithm>
#include <vector>

#include "Arduino.h"

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
using std::max;
using std::min;

class Servo {
public:
    void setPeriodHertz(int hz) { periodHz = hz; }
    int attach(int pin, int lo, int hi) {
        attachedPin = pin;
        minUs = lo;
        maxUs = hi;
        return 0;
    }
    bool attached() const { return attachedPin >= 0; }
    void writeMicroseconds(int us) {
        // the real library clamps to the attach range too
        us = std::min(std::max(us, minUs), maxUs);
        written.push_back(us);
    }

    int periodHz = 0;
    int attachedPin = -1;
    int minUs = 0;
    int maxUs = 0;
    std::vector<int> written;
};
