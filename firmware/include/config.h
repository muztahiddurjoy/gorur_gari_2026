#ifndef CONFIG_H
#define CONFIG_H

//servo
const int SERVO_FREQUENCY_HZ = 50;
const int SERVO_ERROR = 20;
const int STEERING_CENTER_ANGLE = 90 - SERVO_ERROR;
const int STEERING_MAX_ANGLE = 35; // how far the wheels swing either side of centre

//motor
const int MOTOR_PWM_FREQUENCY_HZ = 20000; // above hearing range so the motor stays quiet
const int MOTOR_PWM_RESOLUTION_BITS = 8;  // duty is 0..255
const int MOTOR_MAX_SPEED = 255;

//encoder
const int ENCODER_COUNTS_PER_REV = 1320;   // 11 ppr * 4 edges * 30:1 gearbox, retune for your motor
const int ENCODER_SAMPLE_INTERVAL_MS = 50;

#endif
