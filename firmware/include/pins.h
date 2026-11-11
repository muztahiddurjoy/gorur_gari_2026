#ifndef PINS_H
#define PINS_H

// for the pin type and for RGB_BUILTIN, which the board variant defines
#include <Arduino.h>

//servo pin: MG996R steering servo signal (the servo is powered from its own
//6 V buck, only signal and ground come from the ESP32)
const uint8_t STEERING_SERVO_PIN = 16;
//motor pins: BTS7960 (IBT-2), each with a 10k pull-down on the board so the
//bridge stays off from power-up until MotorController::begin()
const uint8_t MOTOR_RPWM_PIN = 5;  // forward PWM
const uint8_t MOTOR_LPWM_PIN = 6;  // reverse PWM
const uint8_t MOTOR_EN_PIN = 7;    // R_EN and L_EN, tied together
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


//start button: the devkit's own BOOT button, there is no separate one.
// GPIO0 is a strapping pin, so the firmware only ever READS it, after boot,
// with INPUT_PULLUP (pressed = LOW), debounced in software. It is never driven
// as an output and nothing else may be wired to it. Holding it down through a
// reset still drops the chip into the ROM bootloader, as on any devkit.
const uint8_t START_BUTTON_PIN = 0;

// status led pin (plain LED, not the onboard WS2812). GPIO4 -> 220-470R ->
// anode, cathode to GND. It used to be GPIO36, which belongs to the octal
// PSRAM on the N16R8 module (GPIO35-37 must stay unused).
const uint8_t STATUS_LED_PIN = 4;

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