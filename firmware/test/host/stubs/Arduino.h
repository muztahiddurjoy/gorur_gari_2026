// Host-side stand-in for the Arduino core, just enough for the firmware's
// pure-logic modules to compile and run under g++ (see ../run_host_tests.sh).
// Pin writes are recorded in host_pin_log so tests can check their ORDER.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

using std::isnan;
using std::fabs;

template <typename T, typename L, typename H>
inline T constrain(T v, L lo, H hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }

// virtual clock, advanced by the tests
extern unsigned long host_micros;
inline unsigned long micros() { return host_micros; }
inline unsigned long millis() { return host_micros / 1000UL; }

struct HostPinEvent {
    std::string what;  // "en", "pwm"
    int pin;
    uint32_t value;
};
extern std::vector<HostPinEvent> host_pin_log;

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, int level) {
    host_pin_log.push_back({"en", pin, (uint32_t)level});
}
