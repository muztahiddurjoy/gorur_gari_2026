#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>

// Human readable debug output, OFF by default.
//
// The firmware has one USB port and it carries binary MAVLink to the Pi. Any
// text on it lands in the middle of a frame and corrupts the stream, so debug
// text is only compiled in with  -D DEBUG_PRINTS=1  (see platformio.ini), and
// that is ONLY for bench tests WITHOUT the Pi: a serial monitor on the USB
// port instead of mcu_bridge. With the default 0 every DBG_* call compiles to
// nothing (the arguments are still type checked, so nothing rots).
//
// Things the Pi should hear about (drive timeouts, IMU trouble, the I2C
// fallback) go out as MAVLink STATUSTEXT instead, see statusTextf() in
// main.cpp; mcu_bridge logs them as "MCU: ...".
#ifndef DEBUG_PRINTS
#define DEBUG_PRINTS 0
#endif

#define DEBUG_SERIAL Serial

#define DBG_PRINT(x)    do { if (DEBUG_PRINTS) DEBUG_SERIAL.print(x); } while (0)
#define DBG_PRINTLN(x)  do { if (DEBUG_PRINTS) DEBUG_SERIAL.println(x); } while (0)
#define DBG_PRINTF(...) do { if (DEBUG_PRINTS) DEBUG_SERIAL.printf(__VA_ARGS__); } while (0)

const unsigned long DEBUG_SERIAL_BAUD = 115200;  // ignored by native USB

// With a host enumerated on the USB port but nothing reading it, the core's
// USB serial write() waits up to 100 ms per call for buffer space - the whole
// drive-command timeout. 0 = drop instead of waiting.
const uint32_t DEBUG_SERIAL_TX_TIMEOUT_MS = 0;

inline void debugSerialBegin() {
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
    DEBUG_SERIAL.setTxTimeoutMs(DEBUG_SERIAL_TX_TIMEOUT_MS);
#endif
}

#endif // DEBUG_SERIAL_H
