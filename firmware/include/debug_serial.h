#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>

// Human readable logs go to Serial, which is the devkit's native USB port
// (ARDUINO_USB_CDC_ON_BOOT=1 in platformio.ini). The USB port is debug only:
// the MAVLink link to the Raspberry Pi is the UART on GPIO40/41 (Serial1, see
// setup() in main.cpp), so nothing printed here can ever land in the middle of
// a MAVLink frame.
#define DEBUG_SERIAL Serial

const unsigned long DEBUG_SERIAL_BAUD = 115200;  // ignored by native USB, kept for a UART build

// With a host enumerated on the USB port but no terminal reading it, the
// core's USB serial write() waits up to 100 ms per call for buffer space. That
// is the whole drive-command timeout, so debug output is told to drop bytes
// instead of waiting: a lost log line is fine, a stalled loop() is not.
const uint32_t DEBUG_SERIAL_TX_TIMEOUT_MS = 0;

inline void debugSerialBegin() {
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
    // Serial is the USB-Serial/JTAG HWCDC here, the only class with this knob
    DEBUG_SERIAL.setTxTimeoutMs(DEBUG_SERIAL_TX_TIMEOUT_MS);
#endif
}

#endif // DEBUG_SERIAL_H
