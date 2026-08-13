// SharedData.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct SharedData {
    // Display strings (written by control task, read by I2C task)
    String speedText;
    String encoderText;
    String steeringText;
    String yawText;          // to be filled by I2C task with heading
    String vel_data;

    // Sensor values (written by I2C task, read by control task if needed)
    float heading;           // yaw angle in degrees

    // Run stopwatch from the ROS2 run_timer node (written by the control task
    // as the MAVLink frames land, read by the I2C task to draw the OLED line).
    // The MCU never advances these itself - the Pi owns the clock, see
    // lib/run_timer.
    uint32_t runTimeMs = 0;         // elapsed milliseconds in the current run
    uint8_t runTimeState = 0;       // RunTimerState: 0 idle, 1 running, 2 stopped
    bool runTimeSeen = false;       // false until the first frame ever arrives
    unsigned long runTimeRxMs = 0;  // millis() of that frame, for the stale check
};

extern SharedData shared;
extern SemaphoreHandle_t sharedMutex;  // protects the whole struct

#endif