#include "sonar_reader.h"

SonarReader::SonarReader(int trig_pin, int echo_pin, unsigned long timeout_us){
    trigPin = trig_pin;
    echoPin = echo_pin;
    timeoutUs = timeout_us;
}

void SonarReader::begin(){
    pinMode(trigPin, OUTPUT);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
}

uint8_t SonarReader::readCm(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long durationUs = pulseIn(echoPin, HIGH, timeoutUs);
    if(durationUs == 0) return 255; // no echo within timeout, treat as out of range

    long cm = durationUs / 58; // standard HC-SR04 round-trip-time-to-cm constant
    if(cm > 255) cm = 255;
    return (uint8_t)cm;
}
