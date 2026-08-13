#include "run_timer.h"

namespace {

// MM:SS.d. Truncated, never rounded, for the same reason run_timer.py
// truncates: a stopwatch reading 1.0 s at 0.96 s is showing a tenth of a
// second that has not happened yet.
String formatElapsed(uint32_t elapsedMs) {
    if (elapsedMs > RUN_TIMER_DISPLAY_MAX_MS) {
        elapsedMs = RUN_TIMER_DISPLAY_MAX_MS;
    }
    uint32_t tenths = elapsedMs / 100;
    uint32_t minutes = tenths / 600;
    uint32_t seconds = (tenths / 10) % 60;
    uint32_t frac = tenths % 10;

    char buf[12];
    snprintf(buf, sizeof(buf), "%02u:%02u.%u", (unsigned)minutes,
             (unsigned)seconds, (unsigned)frac);
    return String(buf);
}

}  // namespace

String runTimerLine(uint32_t elapsedMs, uint8_t state, bool stale) {
    switch (state) {
        case RUN_TIMER_RUNNING:
            // The "?" is the honest answer to "is that number still moving":
            // it is the last one we were given, and nobody has sent another.
            return "Time " + formatElapsed(elapsedMs) + (stale ? " ?" : "");
        case RUN_TIMER_STOPPED:
            // Already frozen by definition, so staleness tells us nothing new.
            return "Time " + formatElapsed(elapsedMs) + " END";
        default:
            // Idle, and anything we do not recognise: show no time rather
            // than a stale one dressed up as the current run.
            return "Time --:--.-";
    }
}
