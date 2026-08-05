#ifndef SONAR_READER_H
#define SONAR_READER_H

#include <Arduino.h>

// what a sonar reports when it has nothing to say: either it is disabled, or
// no echo came back before the timeout. the ROS2 side turns this into "max
// range" rather than a real measurement.
const uint8_t SONAR_NO_READING_CM = 255;

// single HC-SR04 style ultrasonic sensor. readCm() blocks for at most
// timeout_us while it waits for the echo pulse, then returns the distance
// clamped to fit the uint8_t "cm" field in the mcu_to_ros2 mavlink message.
// a reader constructed with enabled=false touches no pins and never blocks,
// so an unplugged sonar costs nothing.
class SonarReader {
private:
    int trigPin;
    int echoPin;
    unsigned long timeoutUs;
    bool enabled;
public:
    SonarReader(int trig_pin, int echo_pin, unsigned long timeout_us, bool is_enabled);
    void begin();
    uint8_t readCm();
    bool isEnabled();
};

#endif // SONAR_READER_H
