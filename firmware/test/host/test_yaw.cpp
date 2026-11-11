// Host tests for lib/heading/yaw_integrator.h: the gyro Z -> heading maths.

#include <cmath>

#include "host_test.h"
#include "yaw_integrator.h"

int host_failures = 0;
int host_checks = 0;

namespace {

const float SIGN_Z_UP = -1.0f;  // IMU_YAW_SIGN for a Z-up mounted board
const float MAX_GAP = 0.12f;

// feed `seconds` of a constant sensor rate at `hz`, starting at t0
uint32_t spin(YawIntegrator &y, float rateDps, float seconds, float hz, uint32_t t0) {
    uint32_t stepUs = (uint32_t)lroundf(1e6f / hz);
    uint32_t n = (uint32_t)lroundf(seconds * hz);
    uint32_t t = t0;
    for (uint32_t i = 0; i <= n; i++) {
        y.addSample(rateDps, t);
        t += stepUs;
    }
    return t - stepUs;
}

void test_wrap() {
    CHECK_NEAR(wrapDeg180(0.0f), 0.0f, 1e-6);
    CHECK_NEAR(wrapDeg180(180.0f), 180.0f, 1e-6);
    CHECK_NEAR(wrapDeg180(-180.0f), 180.0f, 1e-6);  // (-180, 180]
    CHECK_NEAR(wrapDeg180(190.0f), -170.0f, 1e-4);
    CHECK_NEAR(wrapDeg180(-190.0f), 170.0f, 1e-4);
    CHECK_NEAR(wrapDeg180(725.0f), 5.0f, 1e-3);
}

void test_counter_clockwise_sensor_rate_is_negative_heading() {
    // a Z-up gyro reads +90 deg/s turning counter-clockwise; the heading
    // (clockwise positive, like the BNO055 compass) must go DOWN by 90
    YawIntegrator y(SIGN_Z_UP, MAX_GAP);
    spin(y, 90.0f, 1.0f, 416.0f, 1000);
    CHECK_NEAR(y.yawDeg(), -90.0f, 0.05);
    CHECK_NEAR(y.rateDps(), -90.0f, 1e-4);
}

void test_wraps_past_180() {
    YawIntegrator y(SIGN_Z_UP, MAX_GAP);
    uint32_t t = spin(y, -100.0f, 2.0f, 416.0f, 0);  // clockwise 200 deg
    CHECK_NEAR(y.yawDeg(), -160.0f, 0.1);            // 200 wraps to -160
    // keep turning clockwise at 145 deg/s for another ~2 s, continuing the
    // same stream. Exact expectation from the sample spacing: 832 intervals
    // of 2404 us at 100, one joining interval at (100+145)/2, 832 at 145.
    spin(y, -145.0f, 2.0f, 416.0f, t + 2404);
    const double dt = 2404e-6;
    double expected = 832 * dt * 100.0 + dt * 122.5 + 832 * dt * 145.0 - 360.0;
    CHECK_NEAR(y.yawDeg(), expected, 0.02);          // ~130.3
    CHECK(y.yawDeg() > -180.0f && y.yawDeg() <= 180.0f);
}

void test_gap_longer_than_max_is_not_integrated() {
    YawIntegrator y(SIGN_Z_UP, MAX_GAP);
    y.addSample(-50.0f, 0);
    y.addSample(-50.0f, 500000);  // 0.5 s later: a stall, not integrated
    CHECK_NEAR(y.yawDeg(), 0.0f, 1e-6);
    y.addSample(-50.0f, 600000);  // 0.1 s later: integrated, 5 deg
    CHECK_NEAR(y.yawDeg(), 5.0f, 1e-3);
}

void test_restart_drops_the_interval() {
    YawIntegrator y(SIGN_Z_UP, MAX_GAP);
    y.addSample(-50.0f, 0);
    y.restart();                  // e.g. a fault in between
    y.addSample(-50.0f, 100000);  // would have been 5 deg
    CHECK_NEAR(y.yawDeg(), 0.0f, 1e-6);
    y.addSample(-50.0f, 200000);
    CHECK_NEAR(y.yawDeg(), 5.0f, 1e-3);
}

void test_reset() {
    YawIntegrator y(SIGN_Z_UP, MAX_GAP);
    spin(y, -30.0f, 1.0f, 416.0f, 0);
    CHECK(std::fabs(y.yawDeg()) > 1.0f);
    y.reset();
    CHECK_NEAR(y.yawDeg(), 0.0f, 1e-6);
    y.addSample(-30.0f, 2000000);  // first sample after reset: time base only
    CHECK_NEAR(y.yawDeg(), 0.0f, 1e-6);
}

void test_micros_rollover() {
    YawIntegrator y(SIGN_Z_UP, MAX_GAP);
    uint32_t t = 0xFFFFFFFFu - 50000u;
    y.addSample(-100.0f, t);
    y.addSample(-100.0f, t + 100000u);  // wraps past 0
    CHECK_NEAR(y.yawDeg(), 10.0f, 1e-3);
}

void test_uneven_sampling_like_oled_stalls() {
    // 416 Hz with a 25 ms hole every 200 ms (an OLED frame on the bus), the
    // rate ramping 0 -> 180 deg/s over 1 s: the trapezoidal rule should land
    // close to the true 90 deg
    YawIntegrator y(+1.0f, MAX_GAP);
    const uint32_t step = 2404;  // 416 Hz
    for (uint32_t t = 0; t <= 1000000; t += step) {
        if ((t % 200000) < 25000 && t > 0) continue;  // stalled
        float rate = 180.0f * (float)t / 1e6f;
        y.addSample(rate, t);
    }
    CHECK_NEAR(y.yawDeg(), 90.0f, 0.5);
}

}  // namespace

int main() {
    RUN(test_wrap);
    RUN(test_counter_clockwise_sensor_rate_is_negative_heading);
    RUN(test_wraps_past_180);
    RUN(test_gap_longer_than_max_is_not_integrated);
    RUN(test_restart_drops_the_interval);
    RUN(test_reset);
    RUN(test_micros_rollover);
    RUN(test_uneven_sampling_like_oled_stalls);
    std::printf("yaw: %d checks, %d failures\n", host_checks, host_failures);
    return host_failures == 0 ? 0 : 1;
}
