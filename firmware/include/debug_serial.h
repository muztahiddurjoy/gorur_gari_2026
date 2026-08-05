#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>

// Serial (native USB CDC, the /dev/ttyACM0 mcu_bridge.py opens) carries nothing
// but binary MAVLink frames. Any stray text there lands in the middle of a
// packet and desyncs the ROS2 parser, so human readable logs go to UART0
// instead, which comes out of the devkit's other (CH343) usb socket.
#if ARDUINO_USB_CDC_ON_BOOT
#define DEBUG_SERIAL Serial0
#else
#define DEBUG_SERIAL Serial
#endif

const unsigned long DEBUG_SERIAL_BAUD = 115200;

#endif // DEBUG_SERIAL_H
