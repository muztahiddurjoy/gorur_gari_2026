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
// A and B are deliberately the reverse of the silkscreen: wired the other way
// the quadrature decoder counts DOWN when the car drives forward, which flips
// the sign of every distance the odometry nodes integrate.
const uint8_t ENCODER_A_PIN = 2;
const uint8_t ENCODER_B_PIN = 1;
// I²C pins
const uint8_t I2C_SDA = 8;
const uint8_t I2C_SCL = 9;


//button pin
const uint8_t BUTTON_PIN = 42;

// status led pin (plain LED, not the onboard WS2812 on GPIO48).
// GPIO36 is free on the DevKitC-1 header and clashes with nothing else here:
// it is not a strapping pin (0/3/45/46), not the native USB pair (19/20), not
// UART0 (43/44) and not wired to the SPI flash (26-32). Note that on modules
// with octal PSRAM (N8R8/N16R8) GPIO33-37 belong to the PSRAM die - this build
// never enables PSRAM, but move the LED to 37 or 45 if you fit such a module
// and see trouble.
const uint8_t STATUS_LED_PIN = 36;

// sonar pins (HC-SR04 style), see firmware/pin-map.md
const uint8_t SONAR_FRONT_TRIG_PIN = 16;
const uint8_t SONAR_FRONT_ECHO_PIN = 17;
const uint8_t SONAR_LEFT_TRIG_PIN = 18;
const uint8_t SONAR_LEFT_ECHO_PIN = 21;
const uint8_t SONAR_RIGHT_TRIG_PIN = 38;
const uint8_t SONAR_RIGHT_ECHO_PIN = 39;
const uint8_t SONAR_REAR_TRIG_PIN = 40;
const uint8_t SONAR_REAR_ECHO_PIN = 41;

#endif