// SharedData.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// What loop() is doing with the drivetrain, for the OLED.
enum DriveState : uint8_t {
    DRIVE_WAITING = 0,  // no valid drive command since boot: braked
    DRIVE_ACTIVE,       // valid drive commands arriving: motor follows them
    DRIVE_STOPPED,      // commands stopped for CMD_TIMEOUT_MS: braked, centred
};

// Everything the two cores exchange, behind one mutex.
//
// Raw numbers only: whoever reads formats them on its own time, so nobody
// builds Strings (heap allocations) while holding the lock. loop() on core 1
// only ever TRIES the lock (timeout 0) and skips the exchange for one pass if
// the I2C task happens to hold it, so it can never be held up by the I2C side.
struct SharedData {
    // ---- written by loop() (core 1), read by the I2C task (core 0) ----
    int motorPercent = 0;            // signed output actually on the bridge
    uint8_t steeringAngle = 90;      // last commanded wire angle
    long encoderCount = 0;
    uint8_t driveState = DRIVE_WAITING;

    // Run stopwatch from the ROS2 run_timer node, as the MAVLink frames land.
    // The MCU never advances these itself - the Pi owns the clock, see
    // lib/run_timer.
    uint32_t runTimeMs = 0;         // elapsed milliseconds in the current run
    uint8_t runTimeState = 0;       // RunTimerState: 0 idle, 1 running, 2 stopped
    bool runTimeSeen = false;       // false until the first frame ever arrives
    unsigned long runTimeRxMs = 0;  // millis() of that frame, for the stale check

    // ---- written by the I2C task (core 0), read by loop() (core 1) ----
    float heading = 0.0f;           // yaw angle in degrees
};

extern SharedData shared;
extern SemaphoreHandle_t sharedMutex;  // protects the whole struct

#endif
