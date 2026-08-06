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
};

extern SharedData shared;
extern SemaphoreHandle_t sharedMutex;  // protects the whole struct

#endif