#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>

// MAVLINK_SERIAL (UART1, out to the USB-to-TTL adapter mcu_bridge.py opens -
// see mav_serial.h) carries nothing but binary MAVLink frames. Any stray text
// there lands in the middle of a packet and desyncs the ROS2 parser, so human
// readable logs go to UART0 instead, which comes out of the devkit's other
// (CH343) usb socket. The native USB CDC port is free now that the link has
// moved off it, but logs stay here: the ROM bootloader banner already comes out
// of UART0, so this is the port you watch during a bring-up anyway.
#if ARDUINO_USB_CDC_ON_BOOT
#define DEBUG_SERIAL Serial0
#else
#define DEBUG_SERIAL Serial
#endif

const unsigned long DEBUG_SERIAL_BAUD = 115200;

#endif // DEBUG_SERIAL_H
