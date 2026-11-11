// Host tests for lib/motor_control (BTS7960 driver). Built and run by
// run_host_tests.sh against stub Arduino/ESP32PWM headers that log every
// pin write, so these check what the bridge pins actually see and in what
// order - the part that matters for the hardware.

#include <cmath>
#include <map>

#include "Arduino.h"
#include "host_test.h"
#include "motor_control.h"

unsigned long host_micros = 0;
std::vector<HostPinEvent> host_pin_log;
int host_failures = 0;
int host_checks = 0;

namespace {

const uint8_t RPWM = 5, LPWM = 6, EN = 7;
const uint32_t FULL = 255;  // 8 bit duty

struct PinState {
    std::map<int, uint32_t> level;
};

// Replays the log from the start and checks the invariant at EVERY write:
// RPWM and LPWM are never non-zero at the same time.
bool neverBothDriving() {
    PinState s;
    for (const auto &e : host_pin_log) {
        s.level[e.pin] = e.value;
        if (s.level[RPWM] != 0 && s.level[LPWM] != 0) return false;
    }
    return true;
}

PinState finalState() {
    PinState s;
    for (const auto &e : host_pin_log) s.level[e.pin] = e.value;
    return s;
}

// index of the first write to `pin` with `value` at or after `from`, or -1
int findWrite(int pin, uint32_t value, size_t from = 0) {
    for (size_t i = from; i < host_pin_log.size(); i++)
        if (host_pin_log[i].pin == pin && host_pin_log[i].value == value) return (int)i;
    return -1;
}

MotorController freshMotor(float slewPercent) {
    host_pin_log.clear();
    host_micros = 1000000;
    MotorController m(RPWM, LPWM, EN);
    m.begin(20000, 8, slewPercent);
    return m;
}

void step(MotorController &m, unsigned long us, int times = 1) {
    for (int i = 0; i < times; i++) {
        host_micros += us;
        m.update();
    }
}

void test_begin_leaves_it_braked() {
    MotorController m = freshMotor(500.0f);
    PinState s = finalState();
    CHECK(s.level[RPWM] == 0);
    CHECK(s.level[LPWM] == 0);
    CHECK(s.level[EN] == HIGH);   // brake = both PWM 0, EN high
    CHECK(m.output() == 0.0f);
    // EN went low before the PWM pins were attached/zeroed, then high (brake)
    CHECK(host_pin_log.front().pin == EN && host_pin_log.front().value == LOW);
}

void test_forward_write_order() {
    MotorController m = freshMotor(0.0f);  // no ramp: one update reaches target
    size_t mark = host_pin_log.size();
    m.setTarget(0.5f);
    step(m, 1000);
    int l0 = findWrite(LPWM, 0, mark);
    int en = findWrite(EN, HIGH, mark);
    int r = -1;
    for (size_t i = mark; i < host_pin_log.size(); i++)
        if (host_pin_log[i].pin == RPWM && host_pin_log[i].value > 0) { r = (int)i; break; }
    // LPWM was already 0 and EN already high (brake), so the cached writes may
    // be skipped - but nothing may come before RPWM rises that contradicts it
    CHECK(r >= 0);
    if (l0 >= 0) CHECK(l0 < r);
    if (en >= 0) CHECK(en < r);
    PinState s = finalState();
    CHECK(s.level[LPWM] == 0 && s.level[EN] == HIGH);
    CHECK(s.level[RPWM] == 128);  // 0.5 * 255 rounded
    CHECK(neverBothDriving());
}

void test_reverse_after_forward_passes_through_zero() {
    MotorController m = freshMotor(0.0f);
    m.setTarget(1.0f);
    step(m, 1000);
    CHECK(finalState().level[RPWM] == FULL);
    size_t mark = host_pin_log.size();
    m.setTarget(-1.0f);
    step(m, 1000);  // first update after the flip must land on zero (brake)
    PinState s = finalState();
    CHECK(s.level[RPWM] == 0 && s.level[LPWM] == 0 && s.level[EN] == HIGH);
    CHECK(m.output() == 0.0f);
    step(m, 1000);  // then the other side drives
    s = finalState();
    CHECK(s.level[RPWM] == 0 && s.level[LPWM] == FULL);
    // RPWM had to reach 0 before LPWM rose
    int rZero = findWrite(RPWM, 0, mark);
    int lUp = findWrite(LPWM, FULL, mark);
    CHECK(rZero >= 0 && lUp > rZero);
    CHECK(neverBothDriving());
}

void test_slew_rate_limits_the_ramp() {
    MotorController m = freshMotor(500.0f);  // 5.0 full-scale per second
    m.setTarget(1.0f);
    step(m, 10000, 10);  // 100 ms -> 50 %
    CHECK_NEAR(m.output(), 0.5f, 0.01);
    step(m, 10000, 10);  // 200 ms -> full
    CHECK_NEAR(m.output(), 1.0f, 1e-6);
    // full forward -> full reverse takes 400 ms and stops at zero on the way
    m.setTarget(-1.0f);
    bool sawZero = false;
    for (int i = 0; i < 40; i++) {
        step(m, 10000);
        if (m.output() == 0.0f) sawZero = true;
        CHECK(m.output() <= 1.0f && m.output() >= -1.0f);
    }
    CHECK(sawZero);
    CHECK_NEAR(m.output(), -1.0f, 0.06);
    CHECK(neverBothDriving());
}

void test_brake_is_immediate() {
    MotorController m = freshMotor(500.0f);
    m.setTarget(1.0f);
    step(m, 10000, 30);
    CHECK_NEAR(m.output(), 1.0f, 1e-6);
    m.brake();  // no ramp, no update() needed
    PinState s = finalState();
    CHECK(s.level[RPWM] == 0 && s.level[LPWM] == 0 && s.level[EN] == HIGH);
    CHECK(m.output() == 0.0f && m.target() == 0.0f);
    // and it stays stopped: update() has nothing to ramp towards
    step(m, 10000, 5);
    CHECK(finalState().level[RPWM] == 0);
}

void test_coast_disables_the_bridge() {
    MotorController m = freshMotor(0.0f);
    m.setTarget(-0.3f);
    step(m, 1000);
    m.coast();
    PinState s = finalState();
    CHECK(s.level[RPWM] == 0 && s.level[LPWM] == 0 && s.level[EN] == LOW);
    // the next real command re-enables the bridge before driving
    m.setTarget(0.3f);
    step(m, 1000);
    s = finalState();
    CHECK(s.level[EN] == HIGH && s.level[RPWM] > 0);
}

void test_full_command_reaches_full_duty() {
    // the scaling the firmware applies to the int8 wire throttle
    const int COMMAND_MAX = 127;
    MotorController m = freshMotor(0.0f);
    m.setTarget((float)constrain(127, -COMMAND_MAX, COMMAND_MAX) / COMMAND_MAX);
    step(m, 1000);
    CHECK(finalState().level[RPWM] == FULL);   // was ~127/255 on the TB6612 code
    m.setTarget((float)constrain(-128, -COMMAND_MAX, COMMAND_MAX) / COMMAND_MAX);
    step(m, 1000);
    step(m, 1000);
    CHECK(finalState().level[LPWM] == FULL);
    CHECK(m.outputPercent() == -100);
}

void test_out_of_range_and_nan_targets_are_safe() {
    MotorController m = freshMotor(0.0f);
    m.setTarget(7.0f);
    CHECK(m.target() == 1.0f);
    m.setTarget(NAN);
    CHECK(m.target() == 0.0f);
}

}  // namespace

int main() {
    RUN(test_begin_leaves_it_braked);
    RUN(test_forward_write_order);
    RUN(test_reverse_after_forward_passes_through_zero);
    RUN(test_slew_rate_limits_the_ramp);
    RUN(test_brake_is_immediate);
    RUN(test_coast_disables_the_bridge);
    RUN(test_full_command_reaches_full_duty);
    RUN(test_out_of_range_and_nan_targets_are_safe);
    std::printf("motor: %d checks, %d failures\n", host_checks, host_failures);
    return host_failures == 0 ? 0 : 1;
}
