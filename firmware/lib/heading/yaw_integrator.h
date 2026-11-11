#ifndef YAW_INTEGRATOR_H
#define YAW_INTEGRATOR_H

// Turns a stream of gyro Z rates into a heading. Pure arithmetic (no Arduino
// calls), so test/host can check it on a PC.
//
// THE RESULT IS RELATIVE AND DRIFTS. The BOTGyro has no magnetometer, so
// unlike the BNO055 there is no absolute (compass) heading to fall back on:
// yaw is 0 wherever the car pointed when it was last reset, and any bias the
// zero-rate calibration did not remove integrates into a slow, steady drift
// (a fraction of a degree per minute on a well calibrated, still sensor; more
// if it was calibrated while being moved). Reset it before each run.

#include <stdint.h>

// (-180, 180]
static inline float wrapDeg180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

class YawIntegrator {
public:
    // sign: +1 or -1, maps the sensor's Z rate onto the heading convention
    // (clockwise positive, like the BNO055 compass heading the ROS side was
    // written for). maxGapS: two samples further apart than this are not
    // integrated across - the rate in between is unknown.
    YawIntegrator(float sign, float maxGapS)
        : sign(sign), maxGapS(maxGapS), yaw(0.0f), lastRate(0.0f), lastUs(0), haveLast(false) {}

    // heading := yawDeg, and the next sample starts a fresh interval
    void reset(float yawDeg = 0.0f) {
        yaw = wrapDeg180(yawDeg);
        haveLast = false;
    }

    // forget the previous sample (calibration, a fault, any pause in the
    // stream) so the next one is not integrated over the whole gap
    void restart() { haveLast = false; }

    // One fresh gyro sample: rateDps is the sensor's Z rate in deg/s, nowUs
    // the micros() it was read at. Trapezoidal rule between consecutive
    // samples, so an uneven sample spacing (the OLED sharing the bus) costs
    // little accuracy.
    void addSample(float rateDps, uint32_t nowUs) {
        float rate = sign * rateDps;
        if (haveLast) {
            float dt = (float)(uint32_t)(nowUs - lastUs) * 1e-6f;  // rollover safe
            if (dt > 0.0f && dt <= maxGapS) {
                yaw = wrapDeg180(yaw + 0.5f * (rate + lastRate) * dt);
            }
        }
        lastRate = rate;
        lastUs = nowUs;
        haveLast = true;
    }

    float yawDeg() const { return yaw; }          // (-180, 180], clockwise positive
    float rateDps() const { return lastRate; }    // same sign convention

private:
    float sign;
    float maxGapS;
    float yaw;
    float lastRate;
    uint32_t lastUs;
    bool haveLast;
};

#endif // YAW_INTEGRATOR_H
