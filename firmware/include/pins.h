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

// MAVLink link to the Pi, UART1 out through a USB-to-TTL adapter. Cross the
// wires - this RX goes to the adapter's TX and this TX to the adapter's RX -
// and tie the adapter's GND to the board's GND, or neither end sees a level it
// can read. Leave the adapter's VCC alone: the devkit has its own supply and
// back feeding 5V into a 3V3 pin is how you lose a board.
// UART0 (43/44) already belongs to DEBUG_SERIAL and 19/20 are the native USB
// pair, so UART1 on a free pair is the spare port. Careful with the sonar block
// below: front ECHO and left TRIG sit on exactly these two pins, and those two
// HC-SR04s have to be physically unplugged - disabling them in config.h stops
// the firmware touching the pins but does nothing about the sonar driving them
// from its end. The static_asserts in mav_serial.h catch the software half.
const int8_t MAVLINK_UART_RX_PIN = 18; // <- adapter TX
const int8_t MAVLINK_UART_TX_PIN = 17; // -> adapter RX


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

// sonar pins (HC-SR04 style), see firmware/pin-map.md
// GPIO17 (front ECHO) and GPIO18 (left TRIG) are shared with the MAVLink UART
// above. Both of those sonars are disabled in config.h, so the firmware never
// configures or pulses either pin - but the sonar itself has to be off the
// board too, because ECHO is an output and GPIO17 is now the UART's transmit
// pin. Enabling front or left without moving one side or the other fails the
// build rather than handing you a car whose commands stop arriving. See
// firmware/pin-map.md.
const uint8_t SONAR_FRONT_TRIG_PIN = 16;
const uint8_t SONAR_FRONT_ECHO_PIN = 17;
const uint8_t SONAR_LEFT_TRIG_PIN = 18;
const uint8_t SONAR_LEFT_ECHO_PIN = 21;
const uint8_t SONAR_RIGHT_TRIG_PIN = 38;
const uint8_t SONAR_RIGHT_ECHO_PIN = 39;
const uint8_t SONAR_REAR_TRIG_PIN = 40;
const uint8_t SONAR_REAR_ECHO_PIN = 41;

#endif