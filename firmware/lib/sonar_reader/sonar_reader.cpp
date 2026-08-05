#include "sonar_reader.h"

SonarReader::SonarReader(int trig_pin, int echo_pin, unsigned long timeout_us, bool is_enabled){
    trigPin = trig_pin;
    echoPin = echo_pin;
    timeoutUs = timeout_us;
    enabled = is_enabled;
}

void SonarReader::begin(){
    if(!enabled) return; // nothing wired to these pins, leave them alone

    pinMode(trigPin, OUTPUT);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
}

uint8_t SonarReader::readCm(){
    if(!enabled) return SONAR_NO_READING_CM;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long durationUs = pulseIn(echoPin, HIGH, timeoutUs);
    if(durationUs == 0) return SONAR_NO_READING_CM; // no echo within timeout

    long cm = durationUs / 58; // standard HC-SR04 round-trip-time-to-cm constant
    if(cm > SONAR_NO_READING_CM) cm = SONAR_NO_READING_CM;
    return (uint8_t)cm;
}

bool SonarReader::isEnabled(){
    return enabled;
}
