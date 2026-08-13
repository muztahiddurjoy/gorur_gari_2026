#ifndef RUN_TIMER_H
#define RUN_TIMER_H

#include <Arduino.h>

// The run stopwatch line on the OLED.
//
// The MCU does not time the run itself: it cannot see the state machine that
// decides when a run starts, and it has no idea when homing has finished. The
// autonomy/run_timer node on the Pi owns the clock and ships it over MAVLink
// (gorur_gari_ros2_to_mcu_timer_msg), and everything here is presentation -
// turning that number into the one line the panel has room for.
//
// Minutes are the largest unit, matching run_timer's /run_time_str exactly, so
// the screen and the logs never disagree about what the run time was.

// Wire values of the state field in gorur_gari_ros2_to_mcu_timer_msg.
// Keep in step with RUN_TIMER_WIRE_STATE in ros2_ws/controls/controls/mcu_bridge.py.
enum RunTimerState : uint8_t {
    RUN_TIMER_IDLE = 0,     // no run yet, or re-armed for the next one
    RUN_TIMER_RUNNING = 1,  // the clock is going
    RUN_TIMER_STOPPED = 2,  // frozen on the final time
};

// How long the panel keeps trusting the last frame. mcu_bridge sends at 10 Hz,
// so a whole second of silence is the ROS2 side having stopped talking, not a
// dropped frame - and a clock that has quietly stopped updating is worse than
// one that admits it, because it still looks like a run time.
const unsigned long RUN_TIMER_STALE_MS = 1000;

// Longest run the line has room for. A WRO run is a few minutes, so this only
// ever catches a nonsense value, and it catches it by keeping the line the
// width it always is instead of pushing the rest of the row off the panel.
const uint32_t RUN_TIMER_DISPLAY_MAX_MS = 99UL * 60000UL + 59900UL;  // 99:59.9

// The whole line, ready to draw: "Time --:--.-" before a run, "Time 01:23.4"
// during one, "Time 01:23.4 END" after it, and a trailing "?" if the frames
// stopped arriving while the clock was supposed to be going.
String runTimerLine(uint32_t elapsedMs, uint8_t state, bool stale);

#endif
