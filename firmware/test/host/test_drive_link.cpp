// Host tests for lib/drive_link: which frames count as a drive command, and
// the 100 ms watchdog. Frames are built with the generated pack functions and
// pushed byte by byte through the real mavlink_parse_char(), exactly as
// loop() does with Serial1.

#include <cstring>
#include <vector>

#include "drive_link.h"
#include "host_test.h"
#include "rate_limited_log.h"

int host_failures = 0;
int host_checks = 0;

namespace {

using Bytes = std::vector<uint8_t>;

Bytes toWire(const mavlink_message_t &msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
    return Bytes(buf, buf + n);
}

Bytes driveFrame(int8_t throttle, uint8_t steering) {
    mavlink_message_t msg;
    mavlink_msg_gorur_gari_ros2_to_mcu_msg_pack(2, 1, &msg, throttle, steering);
    return toWire(msg);
}

// Feeds bytes through the parser like loop() does; returns how many drive
// commands were accepted, and the last one.
int feed(const Bytes &bytes, DriveCommand *last = nullptr) {
    int accepted = 0;
    for (uint8_t c : bytes) {
        mavlink_message_t msg{};
        mavlink_status_t status{};
        uint8_t framing = mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status);
        DriveCommand cmd;
        if (decodeDriveCommand(framing, msg, cmd)) {
            accepted++;
            if (last) *last = cmd;
        }
    }
    return accepted;
}

void test_valid_drive_command_is_accepted() {
    DriveCommand cmd{};
    CHECK(feed(driveFrame(-75, 120), &cmd) == 1);
    CHECK(cmd.throttle == -75);
    CHECK(cmd.steering == 120);
    // full range values survive
    CHECK(feed(driveFrame(127, 180), &cmd) == 1 && cmd.throttle == 127 && cmd.steering == 180);
    CHECK(feed(driveFrame(-128, 0), &cmd) == 1 && cmd.throttle == -128 && cmd.steering == 0);
    // a zero command is still a drive command (and still refreshes the watchdog)
    CHECK(feed(driveFrame(0, 90), &cmd) == 1 && cmd.throttle == 0);
}

void test_corrupted_frames_are_rejected() {
    Bytes good = driveFrame(50, 90);
    // flip every bit of every byte after the start marker, one at a time
    for (size_t i = 1; i < good.size(); i++) {
        for (int bit = 0; bit < 8; bit++) {
            Bytes bad = good;
            bad[i] ^= (uint8_t)(1u << bit);
            // a corrupted length byte can make the parser wait for more bytes;
            // a clean frame afterwards must still parse on its own
            CHECK(feed(bad) == 0);
            feed(Bytes(300, 0));  // flush any half-parsed state
        }
    }
    CHECK(feed(good) == 1);
}

void test_truncated_frame_is_rejected() {
    Bytes good = driveFrame(50, 90);
    Bytes cut(good.begin(), good.end() - 3);
    CHECK(feed(cut) == 0);
    feed(Bytes(300, 0));
    CHECK(feed(good) == 1);
}

void test_other_messages_are_not_drive_commands() {
    mavlink_message_t msg;
    mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_pack(2, 1, &msg, 1);
    CHECK(feed(toWire(msg)) == 0);
    mavlink_msg_gorur_gari_ros2_to_mcu_timer_msg_pack(2, 1, &msg, 123456, 1);
    CHECK(feed(toWire(msg)) == 0);
    // a common-dialect HEARTBEAT (id 0, CRC_EXTRA 50), like the one the old
    // send_heartbeat() in mcu_bridge would put on the wire
    std::memset(&msg, 0, sizeof(msg));
    char *p = _MAV_PAYLOAD_NON_CONST(&msg);
    p[4] = 2;  // type
    p[8] = 3;  // mavlink_version
    msg.msgid = 0;
    mavlink_finalize_message(&msg, 2, 1, 9, 9, 50);
    CHECK(feed(toWire(msg)) == 0);
    // a STATUSTEXT echoed back
    mavlink_pack_statustext(2, 1, &msg, STATUSTEXT_SEVERITY_INFO, "hello");
    CHECK(feed(toWire(msg)) == 0);
}

void test_drive_id_with_wrong_crc_extra_is_rejected() {
    // what a sender built from a different (stale) XML would produce
    mavlink_message_t msg;
    std::memset(&msg, 0, sizeof(msg));
    char *p = _MAV_PAYLOAD_NON_CONST(&msg);
    p[0] = 40;
    p[1] = 90;
    msg.msgid = MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg;
    mavlink_finalize_message(&msg, 2, 1, 2, 2, (uint8_t)(MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC + 1));
    CHECK(feed(toWire(msg)) == 0);
}

void test_out_of_range_steering_is_rejected() {
    CHECK(feed(driveFrame(10, 181)) == 0);
    CHECK(feed(driveFrame(10, 255)) == 0);
    CHECK(feed(driveFrame(10, 180)) == 1);
}

std::vector<DriveCommand> collect(const Bytes &stream) {
    std::vector<DriveCommand> got;
    for (uint8_t c : stream) {
        mavlink_message_t msg{};
        mavlink_status_t status{};
        uint8_t framing = mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status);
        DriveCommand cmd;
        if (decodeDriveCommand(framing, msg, cmd)) got.push_back(cmd);
    }
    return got;
}

Bytes framesWithNoise(const Bytes &noise) {
    Bytes stream;
    for (int i = 0; i < 20; i++) {
        Bytes f = driveFrame((int8_t)i, 90);
        stream.insert(stream.end(), f.begin(), f.end());
        if (i % 3 == 0) stream.insert(stream.end(), noise.begin(), noise.end());
    }
    return stream;
}

// nothing that was not sent, nothing mangled, nothing repeated or reordered
void checkOnlyRealCommands(const std::vector<DriveCommand> &got) {
    CHECK(got.size() <= 20);
    for (size_t i = 0; i < got.size(); i++) {
        CHECK(got[i].steering == 90);
        CHECK(got[i].throttle >= 0 && got[i].throttle < 20);
        if (i > 0) CHECK(got[i].throttle > got[i - 1].throttle);
    }
}

void test_junk_between_frames_is_skipped() {
    // line noise with no start marker in it: the parser skips it while idle
    std::vector<DriveCommand> got = collect(framesWithNoise({0x00, 0x13, 0x37, 0xAA, 0xFF}));
    checkOnlyRealCommands(got);
    CHECK(got.size() == 20);
}

void test_fake_start_marker_costs_frames_but_nothing_false() {
    // a fake v2 header (0xFD, len 0x13) makes the parser swallow the next ~31
    // bytes - two or three real frames - before its CRC fails. Repeated every
    // third frame, as here, it keeps the parser from ever resyncing, which is
    // the worst case: frames are lost (the watchdog then brakes), but no
    // command that was not sent ever gets through.
    std::vector<DriveCommand> got = collect(framesWithNoise({0x00, 0xFD, 0x13, 0x37, 0xFE, 0xAA}));
    checkOnlyRealCommands(got);
    // once the noise stops, the next clean frame parses
    feed(Bytes(300, 0));
    CHECK(feed(driveFrame(5, 95)) == 1);
}

void test_watchdog_is_dead_until_the_first_command() {
    DriveWatchdog w(100);
    CHECK(!w.alive(0));
    CHECK(!w.alive(5000));
    CHECK(!w.everFed());
}

void test_watchdog_times_out_after_100_ms() {
    DriveWatchdog w(100);
    w.feed(1000);
    CHECK(w.alive(1000));
    CHECK(w.alive(1100));    // exactly 100 ms old: still in force
    CHECK(!w.alive(1101));   // older than 100 ms: brake
    w.feed(1150);            // the next valid command brings it back
    CHECK(w.alive(1200));
}

void test_50hz_with_four_lost_frames_rides_through() {
    // what mcu_bridge does: a frame every 20 ms. Lose four in a row: the gap
    // between the two that arrive is 100 ms, which is still in force.
    DriveWatchdog w(100);
    uint32_t t = 10000;
    w.feed(t);
    for (uint32_t now = t; now <= t + 100; now++) CHECK(w.alive(now));
    // a fifth lost frame (120 ms gap) is a stop
    CHECK(!w.alive(t + 120));
}

void test_watchdog_survives_millis_rollover() {
    DriveWatchdog w(100);
    uint32_t nearWrap = 0xFFFFFFFFu - 30u;
    w.feed(nearWrap);
    CHECK(w.alive(nearWrap + 50u));   // wraps past zero
    CHECK(w.alive(nearWrap + 100u));
    CHECK(!w.alive(nearWrap + 101u));
}

void test_rate_limited_log() {
    RateLimitedLog log(1000);
    CHECK(log.shouldLog(0));
    CHECK(log.suppressed() == 0);
    int printed = 0;
    for (uint32_t t = 100; t < 1000; t += 100) printed += log.shouldLog(t);
    CHECK(printed == 0);
    CHECK(log.shouldLog(1000));
    CHECK(log.suppressed() == 9);
}

}  // namespace

int main() {
    RUN(test_valid_drive_command_is_accepted);
    RUN(test_corrupted_frames_are_rejected);
    RUN(test_truncated_frame_is_rejected);
    RUN(test_other_messages_are_not_drive_commands);
    RUN(test_drive_id_with_wrong_crc_extra_is_rejected);
    RUN(test_out_of_range_steering_is_rejected);
    RUN(test_junk_between_frames_is_skipped);
    RUN(test_fake_start_marker_costs_frames_but_nothing_false);
    RUN(test_watchdog_is_dead_until_the_first_command);
    RUN(test_watchdog_times_out_after_100_ms);
    RUN(test_50hz_with_four_lost_frames_rides_through);
    RUN(test_watchdog_survives_millis_rollover);
    RUN(test_rate_limited_log);
    std::printf("drive_link: %d checks, %d failures\n", host_checks, host_failures);
    return host_failures == 0 ? 0 : 1;
}
