#ifndef DRIVE_LINK_H
#define DRIVE_LINK_H

// The "move only while drive commands keep arriving" rule, kept free of
// Arduino calls so the host tests (test/host) can run it against the real
// MAVLink parser.
//
// Header only on purpose: it is a few lines of logic, and the firmware and the
// host tests then compile exactly the same text.

#include <stdint.h>

#include "mavlink_link.h"

// What a valid drive command carries.
struct DriveCommand {
    int8_t throttle;   // -128..127, +/-MOTOR_COMMAND_MAX is full duty
    uint8_t steering;  // wire angle 0..180, 90 = straight
};

// mcu_bridge clamps the steering angle to 0..180 before it ever reaches the
// wire. A CRC-valid frame carrying more is not something it produced.
const uint8_t DRIVE_STEERING_MAX_DEG = 180;

// True only for a frame that is a real drive command:
//   * mavlink_parse_char() returned MAVLINK_FRAMING_OK, i.e. the checksum
//     (including the message's CRC_EXTRA) and the length checked out. Frames
//     with a bad CRC or a bad signature come back as something else.
//   * it is a gorur_gari_ros2_to_mcu_msg. Connect notices, the run timer,
//     heartbeats, parameter traffic and any other id are not drive commands.
//   * its steering angle is in range.
// Only a frame that passes all three may refresh the DriveWatchdog.
inline bool decodeDriveCommand(uint8_t framingResult, const mavlink_message_t& msg,
                               DriveCommand& out) {
    if (framingResult != MAVLINK_FRAMING_OK) return false;
    if (msg.msgid != MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg) return false;
    DriveCommand cmd;
    cmd.throttle = mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_throttle(&msg);
    cmd.steering = mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_steering(&msg);
    if (cmd.steering > DRIVE_STEERING_MAX_DEG) return false;
    out = cmd;
    return true;
}

// Tracks how old the newest valid drive command is.
//
// alive() is false from boot until the first feed(), and false again as soon
// as the newest valid command is more than timeoutMs old. loop() brakes the
// motor and centres the steering whenever it is false, so the car moves only
// while valid commands keep arriving. All times are millis(); the unsigned
// subtraction keeps it right across the 49-day rollover.
class DriveWatchdog {
public:
    explicit DriveWatchdog(uint32_t timeout_ms)
        : timeoutMs(timeout_ms), lastFeedMs(0), fed(false) {}

    // a valid drive command arrived at nowMs
    void feed(uint32_t nowMs) {
        lastFeedMs = nowMs;
        fed = true;
    }

    bool alive(uint32_t nowMs) const {
        return fed && (uint32_t)(nowMs - lastFeedMs) <= timeoutMs;
    }

    bool everFed() const { return fed; }
    // ms since the newest valid command (meaningless before the first one)
    uint32_t ageMs(uint32_t nowMs) const { return (uint32_t)(nowMs - lastFeedMs); }
    uint32_t timeout() const { return timeoutMs; }

private:
    uint32_t timeoutMs;
    uint32_t lastFeedMs;
    bool fed;
};

#endif // DRIVE_LINK_H
