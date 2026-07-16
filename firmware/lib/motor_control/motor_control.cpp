#include "motor_control.h"

MotorController::MotorController(int pwm_pin, int in_a_pin, int in_b_pin, int standby_pin){
    pwmPin = pwm_pin;
    inAPin = in_a_pin;
    inBPin = in_b_pin;
    standbyPin = standby_pin;
    motorSpeed = 0;
    maxSpeed = 255; // replaced by begin() once the resolution is known
}

void MotorController::begin(int frequency, int resolution){
    pinMode(inAPin, OUTPUT);
    pinMode(inBPin, OUTPUT);
    pinMode(standbyPin, OUTPUT);

    maxSpeed = (1 << resolution) - 1;
    throttle.attachPin(pwmPin, frequency, resolution);

    digitalWrite(standbyPin, HIGH); // driver awake
    coast();
}

// writes the direction pins and the duty in one place so every
// motor state goes through the same path.
void MotorController::hold(int inA, int inB, int duty){
    digitalWrite(inAPin, inA);
    digitalWrite(inBPin, inB);
    throttle.write(duty);
}

void MotorController::to(int speed){
    if(speed > maxSpeed) speed = maxSpeed;
    if(speed < -maxSpeed) speed = -maxSpeed;
    motorSpeed = speed;

    if(speed > 0) hold(HIGH, LOW, speed);
    else if(speed < 0) hold(LOW, HIGH, -speed);
    else brake();
}

void MotorController::forward(int speed){
    to(abs(speed));
}

void MotorController::reverse(int speed){
    to(-abs(speed));
}

// both direction pins high shorts the motor windings and stops it dead.
void MotorController::brake(){
    motorSpeed = 0;
    hold(HIGH, HIGH, 0);
}

// both direction pins low lets the motor spin down on its own.
void MotorController::coast(){
    motorSpeed = 0;
    hold(LOW, LOW, 0);
}

void MotorController::standby(bool sleeping){
    if(sleeping) coast();
    digitalWrite(standbyPin, sleeping ? LOW : HIGH);
}

int MotorController::speed(){
    return motorSpeed;
}

int MotorController::direction(){
    if(motorSpeed > 0) return 1;
    if(motorSpeed < 0) return -1;
    return 0;
}
