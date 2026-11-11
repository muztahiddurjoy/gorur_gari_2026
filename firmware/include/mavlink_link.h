#ifndef MAVLINK_LINK_H
#define MAVLINK_LINK_H

// Everything the firmware needs from MAVLink, included in one place.
//
// Both dialects in lib/mav_msgs (generated from mav_msg/*.xml) are complete,
// self-contained mavlink.h trees guarded by the same MAVLINK_H macro, and each
// one defines the "primary dialect" macros. Order matters:
//
//   * ros2_to_mcu comes FIRST. Its MAVLINK_MESSAGE_CRCS table is the one the
//     parser (mavlink_helpers.h, compiled once) checks INCOMING frames
//     against, and incoming frames are exactly the ros2_to_mcu messages.
//   * mcu_to_ros2 comes second, only for its pack function. The macros it
//     would otherwise redefine are cleared first; none of them is used at
//     runtime (MAVLINK_USE_MESSAGE_INFO is not set).

#include <stdint.h>
#include <string.h>

#include "ros2_to_mcu/mavlink.h"
#undef MAVLINK_H
#undef MAVLINK_PRIMARY_XML_HASH
#undef MAVLINK_MESSAGE_INFO
#undef MAVLINK_MESSAGE_NAMES
#include "mcu_to_ros2/mavlink.h"

// ---------------------------------------------------------------------------
// STATUSTEXT (#253) from the MAVLink common dialect.
//
// Neither of our dialects defines it, and the MAVLink library in lib_deps is
// built by a newer generator than the headers in lib/mav_msgs, so mixing its
// mavlink_msg_statustext.h in is not safe. It is packed here with the same
// helper every generated pack function ends in. Layout, lengths and CRC_EXTRA
// are those of the generated mavlink_msg_statustext.h:
//
//   offset 0  uint8_t  severity        (MAV_SEVERITY)
//   offset 1  char[50] text            (NUL padded, unterminated when full)
//   offset 51 uint16_t id              (v2 extension, 0 = single chunk)
//   offset 53 uint8_t  chunk_seq       (v2 extension)
//
// mcu_bridge decodes it with pymavlink's common dialect, and
// ros2_ws/controls/test/test_mcu_statustext.py round trips these exact bytes.
// ---------------------------------------------------------------------------
const uint32_t MAVLINK_STATUSTEXT_MSG_ID = 253;
const uint8_t MAVLINK_STATUSTEXT_LEN = 54;
const uint8_t MAVLINK_STATUSTEXT_MIN_LEN = 51;  // without the two extension fields
const uint8_t MAVLINK_STATUSTEXT_CRC_EXTRA = 83;
const uint8_t MAVLINK_STATUSTEXT_TEXT_LEN = 50;

// MAV_SEVERITY values from common.xml (RFC 5424 levels)
const uint8_t STATUSTEXT_SEVERITY_CRITICAL = 2;
const uint8_t STATUSTEXT_SEVERITY_ERROR = 3;
const uint8_t STATUSTEXT_SEVERITY_WARNING = 4;
const uint8_t STATUSTEXT_SEVERITY_NOTICE = 5;
const uint8_t STATUSTEXT_SEVERITY_INFO = 6;

// Packs `text` (truncated to 50 bytes) into msg; returns the payload length,
// like the generated *_pack() functions.
static inline uint16_t mavlink_pack_statustext(uint8_t system_id, uint8_t component_id,
                                               mavlink_message_t* msg,
                                               uint8_t severity, const char* text)
{
    char* payload = _MAV_PAYLOAD_NON_CONST(msg);
    memset(payload, 0, MAVLINK_STATUSTEXT_LEN);
    payload[0] = (char)severity;
    memcpy(&payload[1], text, strnlen(text, MAVLINK_STATUSTEXT_TEXT_LEN));
    // id and chunk_seq stay 0: every status text here fits in one chunk
    msg->msgid = MAVLINK_STATUSTEXT_MSG_ID;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    MAVLINK_STATUSTEXT_MIN_LEN,
                                    MAVLINK_STATUSTEXT_LEN,
                                    MAVLINK_STATUSTEXT_CRC_EXTRA);
}

#endif // MAVLINK_LINK_H
