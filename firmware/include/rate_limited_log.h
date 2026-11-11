#ifndef RATE_LIMITED_LOG_H
#define RATE_LIMITED_LOG_H

#include <stdint.h>

// Lets an event be reported at most once per interval, and counts how often it
// happened in between, so a link flapping at 10 Hz prints one line a second
// ("... (+9 more since last report)") instead of flooding the debug port.
// One instance per kind of event. Times are millis().
class RateLimitedLog {
public:
    explicit RateLimitedLog(uint32_t min_interval_ms)
        : intervalMs(min_interval_ms), lastLogMs(0), pending(0), reported(0), logged(false) {}

    // Call every time the event happens. Returns true when it should be
    // printed now; suppressed() then says how many were swallowed before it.
    bool shouldLog(uint32_t nowMs) {
        if (!logged || (uint32_t)(nowMs - lastLogMs) >= intervalMs) {
            reported = pending;
            pending = 0;
            lastLogMs = nowMs;
            logged = true;
            return true;
        }
        pending++;
        return false;
    }

    uint32_t suppressed() const { return reported; }

private:
    uint32_t intervalMs;
    uint32_t lastLogMs;
    uint32_t pending;
    uint32_t reported;
    bool logged;
};

#endif // RATE_LIMITED_LOG_H
