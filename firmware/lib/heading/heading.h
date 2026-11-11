#ifndef HEADING_H
#define HEADING_H

#include <Arduino.h>
#include <Wire.h>
#include <BOTGyro.h>

#include "yaw_integrator.h"

// The IMU: a BOTGyro (ASM330LHH, 6-axis gyro + accelerometer, NO
// magnetometer) on the shared I2C bus, read through the BOTGyro library.
//
//   yaw         integrated from the filtered Z rate (getZ(), deg/s) with a
//               micros() time base, wrapped to (-180, 180], clockwise positive
//               like the BNO055 compass heading the ROS nodes were written for.
//               RELATIVE and slowly DRIFTING: 0 is wherever the car pointed at
//               boot (or the last resetYaw()), see yaw_integrator.h.
//   pitch/roll  BOTGyro's own complementary filter (enableAttitude()).
//
// Everything here runs on the I2C task (core 0), never in loop(): BOTGyro's
// begin() and its built-in fault recovery each block for ~120 ms in delay()
// (sensor reset + settle), which would starve the 100 ms drive timeout if it
// happened on the control core.

enum class ImuStatus : uint8_t {
    NotFound = 0,   // begin() has not succeeded yet, retried every beginRetryMs
    Calibrating,    // zero-rate calibration running: the car must stay still
    Ready,          // calibrated and streaming, yaw is integrating
    Uncalibrated,   // streaming, but no calibration has succeeded: yaw is held
    Fault,          // stopped answering; BOTGyro re-initialises it every second
};

const char *imuStatusName(ImuStatus status);

struct HeadingConfig {
    uint8_t i2cAddress;
    uint32_t i2cClockHz;         // applied by BOTGyro::begin()
    float odrHz;
    int32_t gyroFullScaleDps;
    int32_t accelFullScaleG;
    uint16_t calibrationSamples;
    float attitudeAlpha;         // complementary filter weight on the gyro
    float yawSign;               // sensor Z rate -> clockwise-positive heading
    float maxIntegrationGapS;    // longer gaps between samples are not integrated
    uint32_t beginRetryMs;       // begin() retry period while not found
};

class Heading {
public:
    Heading(TwoWire &wire, const HeadingConfig &config);

    // Call on every pass of the I2C task. Retries begin() while the sensor is
    // missing, runs BOTGyro's state machine (gyro.update()) and integrates yaw
    // on each fresh sample. Calibration is NON-blocking (startCalibration()).
    //
    // calibrationAllowed: no drive command is in force, so the car should be
    // standing still. Only then does an uncalibrated gyro (re)start its
    // zero-rate calibration - at boot, after a disturbed attempt, or after a
    // fault that hit before the first calibration finished.
    void service(bool calibrationAllowed);

    // Heading := 0 from here. Called at boot. (The MAVLink protocol has no
    // message the Pi could trigger it with; see mav_msg/ros2_to_mcu.xml.)
    void resetYaw() { yaw.reset(0.0f); }

    // For the next begin() / re-initialisation (the I2C fallback uses it).
    void setI2CClock(uint32_t hz) { config.i2cClockHz = hz; gyro.setI2CClock(hz); }

    ImuStatus status() const;
    bool calibrated() const { return started && gyro.isCalibrated(); }
    bool everCalibrated() const { return calibratedOnce; }
    bool beginAttempted() const { return attempted; }

    float yawDeg() const { return yaw.yawDeg(); }
    float yawRateDps() const { return yaw.rateDps(); }
    float pitchDeg() const { return gyro.getPitch(); }
    float rollDeg() const { return gyro.getRoll(); }
    float accelXg() const { return gyro.getAccelX(); }
    float accelYg() const { return gyro.getAccelY(); }
    float accelZg() const { return gyro.getAccelZ(); }
    float rateXDps() const { return gyro.getX(); }
    float rateYDps() const { return gyro.getY(); }
    float rateZDps() const { return gyro.getZ(); }
    uint16_t sampleRateHz() const { return started ? gyro.getSampleRate() : 0; }
    float biasZDps() const { return gyro.getBiasZ(); }
    float noiseZDps() const { return gyro.getNoiseZ(); }

    // Diagnostics for the I2C clock fallback (see main.cpp):
    // times the sensor has gone into Fault, and begin() attempts that failed
    // while something still ACKed the address (present, but not answering
    // properly - as opposed to simply not plugged in).
    uint32_t faultCount() const { return faults; }
    uint32_t failedBeginsWhilePresent() const { return presentButFailed; }
    uint8_t address() const { return config.i2cAddress; }

private:
    TwoWire &wire;
    HeadingConfig config;
    BOTGyro gyro;
    YawIntegrator yaw;

    bool started;
    bool attempted;
    bool calibratedOnce;
    unsigned long lastBeginAttemptMs;
    ImuStatus lastStatus;
    uint32_t faults;
    uint32_t presentButFailed;

    void tryBegin(unsigned long nowMs);
    bool addressAcks();
    void logTransition(ImuStatus from, ImuStatus to);
};

#endif
