// Host tests for lib/steering_control: the wire angle -> pulse map, and the
// guarantee that no wire value can push the servo past its limits.

#include "Arduino.h"
#include "ESP32Servo.h"
#include "host_test.h"
#include "steering_control.h"

unsigned long host_micros = 0;
std::vector<HostPinEvent> host_pin_log;
int host_failures = 0;
int host_checks = 0;

namespace {

void test_centre_on_begin() {
    SteeringController s(16);
    CHECK(s.begin(50, 1100, 1500, 1900));
    CHECK(s.getAngle() == 90);
    CHECK(s.getMicroseconds() == 1500);
}

void test_map_hits_the_three_calibration_points() {
    SteeringController s(16);
    s.begin(50, 1100, 1500, 1900);
    s.to(0);
    CHECK(s.getMicroseconds() == 1100);
    s.to(90);
    CHECK(s.getMicroseconds() == 1500);
    s.to(180);
    CHECK(s.getMicroseconds() == 1900);
    s.to(45);
    CHECK(s.getMicroseconds() == 1300);
}

void test_asymmetric_linkage() {
    // centre trimmed off the middle: each side keeps its own travel
    SteeringController s(16);
    s.begin(50, 1000, 1350, 1900);
    s.to(0);
    CHECK(s.getMicroseconds() == 1000);
    s.to(90);
    CHECK(s.getMicroseconds() == 1350);
    s.to(180);
    CHECK(s.getMicroseconds() == 1900);
}

void test_no_wire_value_leaves_the_limits() {
    SteeringController s(16);
    s.begin(50, 1100, 1500, 1900);
    for (int a = -300; a <= 600; a++) {
        s.to(a);
        CHECK(s.getMicroseconds() >= 1100 && s.getMicroseconds() <= 1900);
        CHECK(s.getAngle() <= 180);
    }
}

void test_reversed_servo_by_swapping_min_and_max() {
    SteeringController s(16);
    s.begin(50, 1900, 1500, 1100);  // wire 0 -> 1900, 180 -> 1100
    s.to(0);
    CHECK(s.getMicroseconds() == 1900);
    s.to(180);
    CHECK(s.getMicroseconds() == 1100);
    for (int a = -50; a <= 300; a++) {
        s.to(a);
        CHECK(s.getMicroseconds() >= 1100 && s.getMicroseconds() <= 1900);
    }
}

void test_centre_outside_the_range_is_clamped() {
    SteeringController s(16);
    s.begin(50, 1100, 2500, 1900);
    CHECK(s.getMicroseconds() == 1900);
}

}  // namespace

int main() {
    RUN(test_centre_on_begin);
    RUN(test_map_hits_the_three_calibration_points);
    RUN(test_asymmetric_linkage);
    RUN(test_no_wire_value_leaves_the_limits);
    RUN(test_reversed_servo_by_swapping_min_and_max);
    RUN(test_centre_outside_the_range_is_clamped);
    std::printf("steering: %d checks, %d failures\n", host_checks, host_failures);
    return host_failures == 0 ? 0 : 1;
}
