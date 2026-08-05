#ifndef SONAR_READER_H
#define SONAR_READER_H

#include <Arduino.h>

// single HC-SR04 style ultrasonic sensor. readCm() blocks for at most
// timeout_us while it waits for the echo pulse, then returns the distance
// clamped to fit the uint8_t "cm" field in the mcu_to_ros2 mavlink message.
class SonarReader {
private:
    int trigPin;
    int echoPin;
    unsigned long timeoutUs;
public:
    SonarReader(int trig_pin, int echo_pin, unsigned long timeout_us);
    void begin();
    uint8_t readCm();
};

#endif // SONAR_READER_H
