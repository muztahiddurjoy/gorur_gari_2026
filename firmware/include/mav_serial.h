#ifndef MAV_SERIAL_H
#define MAV_SERIAL_H

#include <Arduino.h>
#include "pins.h"
#include "config.h"

// The binary MAVLink link to the ROS2 side. It used to be the devkit's native
// USB CDC port; it is now UART1 out to a USB-to-TTL adapter on
// MAVLINK_UART_RX_PIN / MAVLINK_UART_TX_PIN (see pins.h). UART0 stays with
// DEBUG_SERIAL (debug_serial.h), so human readable logs still cannot land in
// the middle of a packet and desync the ROS2 parser.
//
// Two things get easier on a real UART. The ROM bootloader banner goes out on
// UART0, not here, so the link comes up clean instead of with a page of boot
// text in front of the first frame. And a write with nothing listening drains
// at the baud rate and returns, where the CDC port could park the loop for its
// TX timeout whenever the host stopped reading.
#define MAVLINK_SERIAL Serial1

// Must match the mcu_bridge `baudrate` parameter in
// ros2_ws/config/bot_config.yaml - both ends guess nothing. Unlike CDC this is
// a real bit rate, so it is worth knowing what we spend: telemetry is one ~30
// byte frame every SENSOR_TX_INTERVAL_MS, which is around 5% of 115200. There
// is room to grow the message, and room to raise this if it ever runs out -
// just raise it on the Pi in the same commit.
const unsigned long MAVLINK_SERIAL_BAUD = 115200;

// Both buffers are sized for how long loop() can go without draining the port,
// not for how much traffic there is. One pulseIn() timeout is SONAR_ECHO_TIMEOUT_US
// (15 ms, ~170 bytes at 115200) and loop() does other work besides, so the 256
// byte default is thinner than it looks once the sonars are switched on. 1 KB
// buys ~89 ms of silence before a byte is dropped, which no single pass gets
// anywhere near, for 1 KB of the S3's 512.
const size_t MAVLINK_SERIAL_RX_BUFFER = 1024;
// A TX ring buffer at all is the point here: with none, the IDF driver writes
// straight into the 128 byte FIFO and blocks until it fits. Frames are far
// smaller than that today, so this costs nothing and means a burst can never
// stall the control loop.
const size_t MAVLINK_SERIAL_TX_BUFFER = 512;

// Ceiling on bytes handed to the parser in a single loop() pass. Nothing sane
// sends this much - it is the guard for the insane case, a peer at the wrong
// baud rate or a floating RX line, where available() would keep coming back
// non-zero and the encoder, sonars and LEDs would never get another look in.
// Whatever is left stays in the ring buffer and gets picked up next pass.
const size_t MAVLINK_SERIAL_MAX_RX_PER_LOOP = 512;

// --- Compile time pin conflict guard ---------------------------------------
// The UART is the one peripheral here that can lose a pin fight silently: it
// keeps reporting bytes, they are just garbage, and the symptom is a car that
// ignores commands intermittently. Every pin the firmware drives gets checked
// against the two the UART takes, so a future remap on either side is a build
// error at the desk rather than a mystery on the mat.
constexpr bool mavUartUses(uint8_t pin) {
    return pin == (uint8_t)MAVLINK_UART_RX_PIN || pin == (uint8_t)MAVLINK_UART_TX_PIN;
}
// a disabled sonar never has its pins configured and is never pulsed
// (sonar_reader.cpp), so it cannot collide with anything
constexpr bool sonarClearsMavUart(bool enabled, uint8_t trig, uint8_t echo) {
    return !enabled || (!mavUartUses(trig) && !mavUartUses(echo));
}

static_assert(MAVLINK_UART_RX_PIN != MAVLINK_UART_TX_PIN,
    "MAVLink UART RX and TX cannot be the same pin");
static_assert(MAVLINK_UART_RX_PIN >= 0 && MAVLINK_UART_TX_PIN >= 0,
    "MAVLink UART pins must be real GPIOs, not the -1 'leave unchanged' marker");

static_assert(!mavUartUses(19) && !mavUartUses(20),
    "GPIO19/20 are the ESP32-S3 native USB D-/D+ pair, taking them kills the USB port");
static_assert(!mavUartUses(43) && !mavUartUses(44),
    "GPIO43/44 are UART0, which DEBUG_SERIAL owns - pick another pair");
static_assert(!mavUartUses(0) && !mavUartUses(3) && !mavUartUses(45) && !mavUartUses(46),
    "GPIO0/3/45/46 are strapping pins, a UART idling on one can change how the chip boots");
static_assert(!mavUartUses(48),
    "GPIO48 is the onboard WS2812, see RGB_STATUS_LED_PIN in pins.h");

static_assert(!mavUartUses(STEERING_SERVO_PIN) && !mavUartUses(PWM_THROTTLE_PIN) &&
              !mavUartUses(IN_A) && !mavUartUses(IN_B) && !mavUartUses(STANDBY_PIN) &&
              !mavUartUses(ENCODER_A_PIN) && !mavUartUses(ENCODER_B_PIN) &&
              !mavUartUses(I2C_SDA) && !mavUartUses(I2C_SCL) &&
              !mavUartUses(BUTTON_PIN) && !mavUartUses(STATUS_LED_PIN),
    "MAVLink UART shares a pin with the drivetrain, encoder, I2C bus, button or status LED - see firmware/pin-map.md");

static_assert(sonarClearsMavUart(SONAR_FRONT_ENABLED, SONAR_FRONT_TRIG_PIN, SONAR_FRONT_ECHO_PIN),
    "front sonar shares a pin with the MAVLink UART (ECHO is GPIO17) - move the sonar or the UART, see firmware/pin-map.md");
static_assert(sonarClearsMavUart(SONAR_LEFT_ENABLED, SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN),
    "left sonar shares a pin with the MAVLink UART (TRIG is GPIO18) - move the sonar or the UART, see firmware/pin-map.md");
static_assert(sonarClearsMavUart(SONAR_RIGHT_ENABLED, SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN),
    "right sonar shares a pin with the MAVLink UART - move the sonar or the UART, see firmware/pin-map.md");
static_assert(sonarClearsMavUart(SONAR_REAR_ENABLED, SONAR_REAR_TRIG_PIN, SONAR_REAR_ECHO_PIN),
    "rear sonar shares a pin with the MAVLink UART - move the sonar or the UART, see firmware/pin-map.md");

// Bring the link up. Both buffer calls have to happen before begin():
// HardwareSerial refuses to resize a UART that is already running and says so
// only in a log line, on a port nobody is watching. Keeping the order in one
// place means it cannot be got wrong twice.
//
// Nothing here needs the adapter to be plugged in. The IDF sets a pull-up on
// the RX pad, so an unplugged or half wired adapter leaves the line idle high
// and simply delivers no bytes, rather than showering the parser with noise.
inline bool mavlinkSerialBegin() {
    MAVLINK_SERIAL.setRxBufferSize(MAVLINK_SERIAL_RX_BUFFER);
    MAVLINK_SERIAL.setTxBufferSize(MAVLINK_SERIAL_TX_BUFFER);
    MAVLINK_SERIAL.begin(MAVLINK_SERIAL_BAUD, SERIAL_8N1,
                         MAVLINK_UART_RX_PIN, MAVLINK_UART_TX_PIN);
    // false means the IDF driver never installed - a pin the GPIO matrix would
    // not take, or UART1 already claimed. The car is deaf to ROS2 either way,
    // so the caller can at least say so on DEBUG_SERIAL.
    return (bool)MAVLINK_SERIAL;
}

#endif // MAV_SERIAL_H
