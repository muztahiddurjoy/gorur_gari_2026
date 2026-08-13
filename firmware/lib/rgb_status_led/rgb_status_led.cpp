#include "rgb_status_led.h"

// Full brightness triples, scaled down by write(). Purple leans violet instead
// of straight magenta so it never reads as "blue with a tint" next to the
// driving colour - on a diffused WS2812 those two are easy to confuse.
namespace {
struct Colour {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};
constexpr Colour COLOUR_OFF          = {0, 0, 0};
constexpr Colour COLOUR_DISCONNECTED = {255, 0, 0};
constexpr Colour COLOUR_CONNECTED    = {0, 255, 0};
constexpr Colour COLOUR_DRIVING      = {0, 0, 255};
constexpr Colour COLOUR_TURNING      = {160, 0, 255};
}  // namespace

RgbStatusLed::RgbStatusLed(uint8_t pin)
    : ledPin(pin), brightness(255), flashHalfPeriodMs(1), centerAngle(90),
      centerTolerance(0), connected(false), driving(false), turning(false),
      currentState(Disconnected), stateStartMs(0), lastRed(0), lastGreen(0),
      lastBlue(0), everWritten(false) {}

void RgbStatusLed::begin(uint8_t brightnessLevel, unsigned long flashPeriodMs,
                         uint8_t center, uint8_t centerToleranceDeg) {
    brightness = brightnessLevel;
    // half a cycle is what the flash actually counts in, and flashLit() divides
    // by it every pass - clamp it so a mistyped 0 in config.h cannot divide by
    // zero on the very first update()
    flashHalfPeriodMs = flashPeriodMs / 2;
    if (flashHalfPeriodMs == 0) flashHalfPeriodMs = 1;
    centerAngle = center;
    centerTolerance = centerToleranceDeg;

    currentState = Disconnected;
    stateStartMs = millis();
    // neopixelWrite() sets the pin up itself on first use, so there is no
    // pinMode() here. Red goes out now rather than on the first update(), so
    // the pixel never sits on a stale colour from before the reset.
    write(COLOUR_DISCONNECTED.red, COLOUR_DISCONNECTED.green,
          COLOUR_DISCONNECTED.blue);
}

void RgbStatusLed::setConnected(bool isConnected) {
    connected = isConnected;
}

void RgbStatusLed::setDrive(int motorPwm, uint8_t steeringAngle) {
    // any non-zero duty is motion: reverse drives the car just as hard as
    // forward, so testing for > 0 would leave the pixel green while it backs up
    driving = (motorPwm != 0);

    int offset = (int)steeringAngle - (int)centerAngle;
    if (offset < 0) offset = -offset;
    // the tolerance keeps a command that lands a degree off centre from
    // flipping the pixel between blue and purple every frame
    turning = (offset > (int)centerTolerance);
}

RgbStatusLed::State RgbStatusLed::resolveState() const {
    if (driving) return turning ? Turning : Driving;
    return connected ? Connected : Disconnected;
}

bool RgbStatusLed::isFlashing(State s) {
    return s == Driving || s == Turning;
}

bool RgbStatusLed::flashLit() const {
    // Phase is derived from elapsed time instead of toggling a flag on a timer,
    // so a loop() that stalls (a sonar timing out, a long i2c wait) resyncs on
    // the next pass rather than leaving the pixel stuck dark. The subtraction
    // is unsigned, which makes it safe across the millis() rollover.
    unsigned long elapsed = millis() - stateStartMs;
    return ((elapsed / flashHalfPeriodMs) % 2) == 0;
}

void RgbStatusLed::update() {
    State next = resolveState();
    if (next != currentState) {
        bool wasFlashing = isFlashing(currentState);
        currentState = next;
        // Restart the phase when the flashing starts, so the car moving off
        // shows on the spot instead of up to half a period later. Swapping
        // straight from one flashing colour to the other keeps the running
        // phase: steering that sits right on the tolerance edge flips
        // blue/purple every frame, and restarting on each of those would hold
        // the pixel permanently lit - the opposite of "flashing".
        if (!wasFlashing) stateStartMs = millis();
    }

    Colour colour;
    switch (currentState) {
        case Turning:   colour = flashLit() ? COLOUR_TURNING : COLOUR_OFF; break;
        case Driving:   colour = flashLit() ? COLOUR_DRIVING : COLOUR_OFF; break;
        case Connected: colour = COLOUR_CONNECTED; break;
        default:        colour = COLOUR_DISCONNECTED; break;
    }
    write(colour.red, colour.green, colour.blue);
}

void RgbStatusLed::write(uint8_t red, uint8_t green, uint8_t blue) {
    // scaled here rather than in the table above, so brightness stays one knob
    // and the colours stay readable
    red   = (uint8_t)(((uint16_t)red * brightness) / 255);
    green = (uint8_t)(((uint16_t)green * brightness) / 255);
    blue  = (uint8_t)(((uint16_t)blue * brightness) / 255);

    if (everWritten && red == lastRed && green == lastGreen && blue == lastBlue) {
        return;  // already on the wire, and each frame blocks ~30 us
    }
    lastRed = red;
    lastGreen = green;
    lastBlue = blue;
    everWritten = true;
    // from the arduino core (esp32-hal-rgb-led), bit banged over RMT. it takes
    // the RGB_BUILTIN pseudo pin as well as a plain gpio number.
    neopixelWrite(ledPin, red, green, blue);
}
