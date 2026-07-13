#include <Arduino.h>
#include "steering_control.h"
#include "pins.h"
#include "config.h"
SteeringController steer(STEERING_SERVO_PIN);

void setup(){
    steer.begin(SERVO_FREQUENCY_HZ);
}

void loop(){
    for(int angle = 0; angle <= 180; angle++){
        steer.to(angle);
        delay(20);
    }
    for(int angle = 180; angle >= 0; angle--){
        steer.to(angle);
        delay(20);
    }
}