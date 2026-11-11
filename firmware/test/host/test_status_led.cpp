// Host tests for lib/status_led: start countdown > alert pattern > set() level.

#include "Arduino.h"
#include "host_test.h"
#include "status_led.h"

unsigned long host_micros = 0;
std::vector<HostPinEvent> host_pin_log;
int host_failures = 0;
int host_checks = 0;

namespace {

const uint8_t PIN = 4;

int pinLevel() {
    for (auto it = host_pin_log.rbegin(); it != host_pin_log.rend(); ++it)
        if (it->pin == PIN) return (int)it->value;
    return -1;
}

// advance the clock to `ms`, calling update() every millisecond on the way,
// the way loop() calls it (in reality far more often)
void at(StatusLed &led, unsigned long ms) {
    unsigned long now = host_micros / 1000UL;
    while (now < ms) {
        now++;
        host_micros = now * 1000UL;
        led.update();
    }
    host_micros = ms * 1000UL;
    led.update();
}

void test_starts_off_and_follows_set() {
    host_pin_log.clear();
    host_micros = 0;
    StatusLed led(PIN);
    led.begin();
    CHECK(pinLevel() == LOW);
    led.set(true);
    CHECK(pinLevel() == HIGH);
    at(led, 10);
    CHECK(pinLevel() == HIGH);
}

void test_alert_overrides_level_and_gives_it_back() {
    host_pin_log.clear();
    host_micros = 0;
    StatusLed led(PIN);
    led.begin();
    led.set(true);                 // bridge connected
    led.setAlert(200, 100);        // IMU calibrating
    at(led, 50);
    CHECK(pinLevel() == HIGH);
    at(led, 150);
    CHECK(pinLevel() == LOW);
    at(led, 250);
    CHECK(pinLevel() == HIGH);
    led.set(false);                // a level change while the alert runs...
    at(led, 260);
    CHECK(pinLevel() == HIGH);     // ...does not cut into the pattern
    led.setAlert(0, 0);            // calibrated
    at(led, 300);
    CHECK(pinLevel() == LOW);      // back to the last level asked for
}

void test_setting_the_same_alert_does_not_restart_it() {
    host_pin_log.clear();
    host_micros = 0;
    StatusLed led(PIN);
    led.begin();
    led.setAlert(1000, 100);
    at(led, 50);
    CHECK(pinLevel() == HIGH);
    at(led, 500);
    led.setAlert(1000, 100);       // loop() calls this every pass
    at(led, 600);
    CHECK(pinLevel() == LOW);      // still in the dark part of the first period
}

void test_countdown_outranks_the_alert() {
    host_pin_log.clear();
    host_micros = 0;
    StatusLed led(PIN);
    led.begin();
    led.set(true);
    led.setAlert(1000, 100);       // IMU fault blip
    host_micros = 2000 * 1000UL;
    led.blink(3, 1000);            // start button
    CHECK(pinLevel() == HIGH);
    at(led, 2400);
    CHECK(pinLevel() == HIGH);     // countdown lit half, not the alert's dark part
    at(led, 2600);
    CHECK(pinLevel() == LOW);
    at(led, 3000);
    CHECK(pinLevel() == HIGH);     // second cycle
    at(led, 4999);
    CHECK(led.isBlinking());
    at(led, 5000);                 // countdown over (3 x 1000 ms)
    CHECK(!led.isBlinking());
    // the alert takes the LED back: 5000 % 1000 = 0 -> lit part
    CHECK(pinLevel() == HIGH);
    at(led, 5500);
    CHECK(pinLevel() == LOW);
    led.setAlert(0, 0);
    at(led, 5600);
    CHECK(pinLevel() == HIGH);     // and the connected level after that
}

void test_countdown_returns_to_level_set_during_it() {
    host_pin_log.clear();
    host_micros = 0;
    StatusLed led(PIN);
    led.begin();
    led.blink(2, 1000);
    at(led, 700);
    led.set(true);                 // connect notice mid countdown
    CHECK(pinLevel() == LOW);      // recorded, not shown yet
    at(led, 2000);
    CHECK(!led.isBlinking());
    CHECK(pinLevel() == HIGH);
}

}  // namespace

int main() {
    RUN(test_starts_off_and_follows_set);
    RUN(test_alert_overrides_level_and_gives_it_back);
    RUN(test_setting_the_same_alert_does_not_restart_it);
    RUN(test_countdown_outranks_the_alert);
    RUN(test_countdown_returns_to_level_set_during_it);
    std::printf("status_led: %d checks, %d failures\n", host_checks, host_failures);
    return host_failures == 0 ? 0 : 1;
}
