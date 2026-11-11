#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>

#include "config.h"  // MAV_SERIAL

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

// the only USB port there is - the MAVLink link
#define DEBUG_SERIAL MAV_SERIAL

#define DBG_PRINT(x)    do { if (DEBUG_PRINTS) DEBUG_SERIAL.print(x); } while (0)
#define DBG_PRINTLN(x)  do { if (DEBUG_PRINTS) DEBUG_SERIAL.println(x); } while (0)
#define DBG_PRINTF(...) do { if (DEBUG_PRINTS) DEBUG_SERIAL.printf(__VA_ARGS__); } while (0)

#endif // DEBUG_SERIAL_H
