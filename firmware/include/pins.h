#ifndef PINS_H
#define PINS_H

// for the pin type and for RGB_BUILTIN, which the board variant defines
#include <Arduino.h>

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
const uint8_t ENCODER_A_PIN = 1;
const uint8_t ENCODER_B_PIN = 2;
// I²C pins
const uint8_t I2C_SDA = 8;
const uint8_t I2C_SCL = 9;

// UART link to the Raspberry Pi (Serial1, MAVLink 2), through 6N137
// optocouplers to the Pi's GPIO UART (/dev/ttyAMA0). TX/RX are from the
// ESP32's side: GPIO40 drives the Pi's RXD, GPIO41 listens to the Pi's TXD.
const uint8_t PI_UART_TX_PIN = 40;
const uint8_t PI_UART_RX_PIN = 41;


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

// the devkit's own WS2812, the single addressable pixel the
// ESP32-S3-DevKitC-1 puts on GPIO48. RGB_BUILTIN is what the board variant calls it
// (a pseudo pin number, SOC_GPIO_PIN_COUNT + 48, which neopixelWrite() maps
// back to the real gpio); the literal is the fallback for a variant that does
// not define it. Some boards have too little drive voltage on 48 and expect
// the pixel to be rewired to a spare 3V3 capable pin - change this if you fit
// one of those.
#ifdef RGB_BUILTIN
const uint8_t RGB_STATUS_LED_PIN = RGB_BUILTIN;
#else
const uint8_t RGB_STATUS_LED_PIN = 48;
#endif

// No sonars are fitted. Their old pins now belong to the steering servo (16)
// and the UART link to the Pi (40/41); see SONAR_WIRE_NO_READING_CM in config.h
// for what the telemetry still reports in the four sonar fields.

#endif