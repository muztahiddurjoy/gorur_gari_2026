#ifndef RGB_STATUS_LED_H
#define RGB_STATUS_LED_H

#include <Arduino.h>

// The devkit's onboard WS2812 (GPIO48), driven alongside the plain StatusLed.
// The plain LED answers one question - is anything talking to the MCU - and
// owns the start countdown. This one carries the whole picture in colour, so a
// glance from across the table says what the car is doing:
//
//   red              nothing has announced itself over serial since boot
//   green            ROS2 bridge connected, drivetrain idle
//   flashing blue    motor driving, steering centred
//   flashing purple  motor driving with the wheels turned
//
// Motion outranks the link colour: wheels actually turning is the urgent thing
// to see, and the link state is on the plain LED anyway. Reverse counts as
// driving - the sign of the pwm is a direction, not an amount.
//
// Everything here runs from loop() on core 1. neopixelWrite() drives one shared
// RMT channel and is not reentrant, so keep it off the I2C task.
class RgbStatusLed {
public:
    // ordered by priority, highest last - update() takes the highest that fits
    enum State : uint8_t {
        Disconnected = 0,
        Connected,
        Driving,
        Turning,
    };

    explicit RgbStatusLed(uint8_t pin);

    // Lights it red on the spot, so green always means the link came up after
    // this boot (a soft reset leaves the pixel holding whatever colour it had).
    // brightness scales every colour, 0..255 - the raw pixel is painfully
    // bright and this one is 20 cm from the driver's eyes on the bench.
    // flashPeriodMs is one full on+off cycle; anything under 2 ms is treated as
    // 2 ms so the half period never rounds to zero.
    void begin(uint8_t brightness, unsigned long flashPeriodMs,
               uint8_t centerAngle, uint8_t centerToleranceDeg);

    // latched from the ROS2 connect message, the same event the plain LED uses
    void setConnected(bool connected);
    // Fed every loop() from the drivetrain itself rather than from the last
    // serial frame, so anything that moves the car - a bench mode, a command
    // that arrived before the connect notice - still lights it.
    void setDrive(int motorPwm, uint8_t steeringAngle);

    // call every loop() - this is what advances the flash and writes the pixel
    void update();

    State state() const { return currentState; }
    bool isConnected() const { return connected; }

private:
    uint8_t ledPin;
    uint8_t brightness;
    unsigned long flashHalfPeriodMs; // never 0, see begin()
    uint8_t centerAngle;
    uint8_t centerTolerance;

    // the inputs the state is resolved from
    bool connected;
    bool driving;
    bool turning;

    State currentState;
    unsigned long stateStartMs; // when currentState was entered = flash phase 0

    // last bytes actually clocked out. one WS2812 frame is 24 bits of blocking
    // RMT, so nothing goes on the wire unless the colour really changed
    uint8_t lastRed, lastGreen, lastBlue;
    bool everWritten;

    State resolveState() const;
    static bool isFlashing(State s);
    bool flashLit() const;
    void write(uint8_t red, uint8_t green, uint8_t blue);
};

#endif
