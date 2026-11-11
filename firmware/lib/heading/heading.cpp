#include "heading.h"
#include "debug_serial.h"

const char *imuStatusName(ImuStatus status) {
    switch (status) {
        case ImuStatus::NotFound:     return "NOT FOUND";
        case ImuStatus::Calibrating:  return "CALIBRATING";
        case ImuStatus::Ready:        return "READY";
        case ImuStatus::Uncalibrated: return "UNCALIBRATED";
        case ImuStatus::Fault:        return "FAULT";
    }
    return "?";
}

Heading::Heading(TwoWire &wire_bus, const HeadingConfig &cfg)
    : wire(wire_bus), config(cfg), gyro(wire_bus, cfg.i2cAddress),
      yaw(cfg.yawSign, cfg.maxIntegrationGapS), started(false), attempted(false),
      calibratedOnce(false), lastBeginAttemptMs(0), lastStatus(ImuStatus::NotFound),
      faults(0), presentButFailed(0) {
    // all of these only store a value; BOTGyro applies them in begin()
    gyro.setI2CClock(config.i2cClockHz);
    gyro.setODR(config.odrHz);
    gyro.setFullScale(config.gyroFullScaleDps);
    gyro.setAccelFullScale(config.accelFullScaleG);
    gyro.setCalibrationSamples(config.calibrationSamples);
    gyro.setFilterEnabled(true);
    gyro.enableAttitude(true);
    gyro.setAttitudeFilter(config.attitudeAlpha);
    yaw.reset(0.0f);
}

bool Heading::addressAcks() {
    wire.beginTransmission(config.i2cAddress);
    return wire.endTransmission() == 0;
}

void Heading::tryBegin(unsigned long nowMs) {
    attempted = true;
    lastBeginAttemptMs = nowMs;
    // BOTGyro::begin() calls Wire.begin() (a no-op, the bus is already up on
    // our pins) and Wire.setClock(), checks WHO_AM_I, then resets and
    // configures the sensor, blocking ~120 ms. It leaves it streaming
    // uncalibrated; service() starts the calibration.
    if (gyro.begin()) {
        started = true;
        yaw.reset(0.0f);
        DEBUG_SERIAL.printf("[imu] BOTGyro found at 0x%02X, I2C %lu Hz\n",
                            config.i2cAddress, (unsigned long)config.i2cClockHz);
        return;
    }
    if (addressAcks()) {
        presentButFailed++;
        DEBUG_SERIAL.printf("[imu] something answers at 0x%02X but BOTGyro begin() failed, retrying in %lu ms\n",
                            config.i2cAddress, (unsigned long)config.beginRetryMs);
    } else {
        DEBUG_SERIAL.printf("[imu] BOTGyro not found at 0x%02X, retrying in %lu ms\n",
                            config.i2cAddress, (unsigned long)config.beginRetryMs);
    }
}

void Heading::service(bool calibrationAllowed) {
    unsigned long now = millis();

    if (!started) {
        // not found yet (unplugged, still in its own power-on reset, or a
        // brownout at boot): keep trying, never give up
        if (!attempted || now - lastBeginAttemptMs >= config.beginRetryMs) {
            tryBegin(now);
        }
        if (!started) return;
    }

    // BOTGyro's state machine: one sample per call when data is ready, the
    // non-blocking calibration, and its own re-init attempt once a second
    // while in Fault
    bool fresh = gyro.update();

    if (fresh && gyro.isCalibrated()) {
        yaw.addSample(gyro.getZ(), micros());
    }
    // calibrating, faulted or uncalibrated: no integration, and the next good
    // sample starts a fresh interval instead of spanning the gap
    if (!gyro.isRunning() || !gyro.isCalibrated()) {
        yaw.restart();
    }

    // streaming but not calibrated, and the car is meant to be still:
    // (re)start the zero-rate calibration. BOTGyro gives up after five
    // disturbed attempts and goes back to streaming uncalibrated, which lands
    // here again - so it keeps trying for as long as the car stands still.
    if (gyro.isRunning() && !gyro.isCalibrated() && calibrationAllowed) {
        gyro.startCalibration();
    }

    if (gyro.isCalibrated()) calibratedOnce = true;

    ImuStatus now_status = status();
    if (now_status != lastStatus) {
        if (now_status == ImuStatus::Fault) faults++;
        logTransition(lastStatus, now_status);
        lastStatus = now_status;
    }
}

ImuStatus Heading::status() const {
    if (!started) return ImuStatus::NotFound;
    switch (gyro.getState()) {
        case BOTGyro::State::Calibrating: return ImuStatus::Calibrating;
        case BOTGyro::State::Fault:       return ImuStatus::Fault;
        case BOTGyro::State::Running:
            return gyro.isCalibrated() ? ImuStatus::Ready : ImuStatus::Uncalibrated;
        case BOTGyro::State::Idle:
        default:                          return ImuStatus::NotFound;
    }
}

void Heading::logTransition(ImuStatus from, ImuStatus to) {
    if (to == ImuStatus::Ready && from == ImuStatus::Calibrating) {
        DEBUG_SERIAL.printf("[imu] calibrated: bias z %.3f dps, noise z %.3f dps\n",
                            gyro.getBiasZ(), gyro.getNoiseZ());
    } else if (to == ImuStatus::Uncalibrated && from == ImuStatus::Calibrating) {
        DEBUG_SERIAL.println("[imu] calibration disturbed (car moved?), yaw held until it succeeds");
    } else {
        DEBUG_SERIAL.printf("[imu] %s -> %s\n", imuStatusName(from), imuStatusName(to));
    }
}
