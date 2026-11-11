#ifndef MAV_SERIAL_H
#define MAV_SERIAL_H

// The MAVLink link to the Raspberry Pi: native USB CDC on the devkit's "USB"
// port (MAV_SERIAL, config.h). Everything here is non-blocking: the car boots,
// runs the IMU and enforces the 100 ms drive timeout whether or not a host is
// plugged in or has the port open.

#include <Arduino.h>

#include "config.h"

#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
#define MAV_SERIAL_IS_HWCDC 1
#else
#define MAV_SERIAL_IS_HWCDC 0
#endif

// Call once in setup(). Returns at once: there is no waiting for the host
// (no `while (!Serial)`), the port simply starts working when one shows up.
inline void mavSerialBegin() {
#if MAV_SERIAL_IS_HWCDC
    // before begin(), which only allocates buffers that are not set yet
    MAV_SERIAL.setRxBufferSize(MAV_SERIAL_RX_BUFFER_BYTES);
    MAV_SERIAL.setTxBufferSize(MAV_SERIAL_TX_BUFFER_BYTES);
#endif
    MAV_SERIAL.begin(MAV_SERIAL_BAUD);
#if MAV_SERIAL_IS_HWCDC
    MAV_SERIAL.setTxTimeoutMs(MAV_SERIAL_TX_TIMEOUT_MS);
#endif
}

// Writes a whole MAVLink frame or nothing. If the TX buffer cannot take all of
// it (Pi not reading, port closed) the frame is dropped: a half frame would
// only make the Pi's parser throw away the next one too, and waiting would
// stall loop(). Only loop()/setup() write to the link, so nothing can take the
// space between the check and the write.
inline bool mavWriteFrame(const uint8_t *buf, size_t len) {
    if (MAV_SERIAL.availableForWrite() < (int)len) return false;
    return MAV_SERIAL.write(buf, len) == len;
}

// Is a USB host on the bus right now? The core watches the USB start-of-frame
// packets a host sends every millisecond and reports "gone" a few ms after
// they stop (cable pulled, Pi powered off or rebooting). It costs one flag
// read. It cannot tell that an application merely closed the port; the
// drive-command timeout covers that.
inline bool mavHostPresent() {
#if MAV_SERIAL_IS_HWCDC
    return HWCDC::isPlugged();
#else
    return true;
#endif
}

#endif // MAV_SERIAL_H
