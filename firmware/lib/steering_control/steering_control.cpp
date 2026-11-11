#include "steering_control.h"

SteeringController::SteeringController(int pin){
    servoPin = pin;
    steeringAngle = 90; // default angle
}

void SteeringController::begin(int frequency){
    // ESP32PWM::allocateTimer() is called once, at the top of setup()
    steer.setPeriodHertz(frequency);
    steer.attach(servoPin, 500, 2400);
}

void SteeringController::to(int angle){
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    steeringAngle = angle;
    steer.write(steeringAngle-servoCorrection);
}

uint8_t SteeringController::getAngle(){
    return steeringAngle;
}