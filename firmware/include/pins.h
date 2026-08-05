#ifndef PINS_H
#define PINS_H

//servo pins
const uint8_t STEERING_SERVO_PIN = 10;
//motor pins
const uint8_t PWM_THROTTLE_PIN = 4;
const uint8_t IN_A = 5;
const uint8_t IN_B = 6;
const uint8_t STANDBY_PIN = 7;
//encoder pin
const uint8_t ENCODER_A_PIN = 1;
const uint8_t ENCODER_B_PIN = 2;
// I²C pins
const uint8_t I2C_SDA = 8;
const uint8_t I2C_SCL = 9;

// sonar pins (HC-SR04 style), see firmware/pin-map.md
const uint8_t SONAR_FRONT_TRIG_PIN = 16;
const uint8_t SONAR_FRONT_ECHO_PIN = 17;
const uint8_t SONAR_LEFT_TRIG_PIN = 18;
const uint8_t SONAR_LEFT_ECHO_PIN = 21;
const uint8_t SONAR_RIGHT_TRIG_PIN = 38;
const uint8_t SONAR_RIGHT_ECHO_PIN = 39;
const uint8_t SONAR_REAR_TRIG_PIN = 40;
const uint8_t SONAR_REAR_ECHO_PIN = 41;

// onboard ws2812, "Status LED 1" in pin-map.md. same pin the esp32-s3-devkitc-1
// variant exposes as RGB_BUILTIN / PIN_NEOPIXEL.
const uint8_t STATUS_LED_PIN = 48;

#endif