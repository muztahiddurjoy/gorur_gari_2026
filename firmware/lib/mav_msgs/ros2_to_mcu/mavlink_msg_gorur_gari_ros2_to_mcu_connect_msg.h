#pragma once
// MESSAGE gorur_gari_ros2_to_mcu_connect_msg PACKING

#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg 50003


typedef struct __mavlink_gorur_gari_ros2_to_mcu_connect_msg_t {
 uint8_t connected; /*<  1 = link established, 0 = link going down*/
} mavlink_gorur_gari_ros2_to_mcu_connect_msg_t;

#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN 1
#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN 1
#define MAVLINK_MSG_ID_50003_LEN 1
#define MAVLINK_MSG_ID_50003_MIN_LEN 1

#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC 149
#define MAVLINK_MSG_ID_50003_CRC 149



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_gorur_gari_ros2_to_mcu_connect_msg { \
    50003, \
    "gorur_gari_ros2_to_mcu_connect_msg", \
    1, \
    {  { "connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gorur_gari_ros2_to_mcu_connect_msg_t, connected) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_gorur_gari_ros2_to_mcu_connect_msg { \
    "gorur_gari_ros2_to_mcu_connect_msg", \
    1, \
    {  { "connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gorur_gari_ros2_to_mcu_connect_msg_t, connected) }, \
         } \
}
#endif

/**
 * @brief Pack a gorur_gari_ros2_to_mcu_connect_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param connected  1 = link established, 0 = link going down
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN];
    _mav_put_uint8_t(buf, 0, connected);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
#else
    mavlink_gorur_gari_ros2_to_mcu_connect_msg_t packet;
    packet.connected = connected;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
}

/**
 * @brief Pack a gorur_gari_ros2_to_mcu_connect_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param connected  1 = link established, 0 = link going down
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN];
    _mav_put_uint8_t(buf, 0, connected);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
#else
    mavlink_gorur_gari_ros2_to_mcu_connect_msg_t packet;
    packet.connected = connected;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
#endif
}

/**
 * @brief Pack a gorur_gari_ros2_to_mcu_connect_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param connected  1 = link established, 0 = link going down
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN];
    _mav_put_uint8_t(buf, 0, connected);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
#else
    mavlink_gorur_gari_ros2_to_mcu_connect_msg_t packet;
    packet.connected = connected;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
}

/**
 * @brief Encode a gorur_gari_ros2_to_mcu_connect_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_ros2_to_mcu_connect_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gorur_gari_ros2_to_mcu_connect_msg_t* gorur_gari_ros2_to_mcu_connect_msg)
{
    return mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_pack(system_id, component_id, msg, gorur_gari_ros2_to_mcu_connect_msg->connected);
}

/**
 * @brief Encode a gorur_gari_ros2_to_mcu_connect_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_ros2_to_mcu_connect_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gorur_gari_ros2_to_mcu_connect_msg_t* gorur_gari_ros2_to_mcu_connect_msg)
{
    return mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_pack_chan(system_id, component_id, chan, msg, gorur_gari_ros2_to_mcu_connect_msg->connected);
}

/**
 * @brief Encode a gorur_gari_ros2_to_mcu_connect_msg struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_ros2_to_mcu_connect_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gorur_gari_ros2_to_mcu_connect_msg_t* gorur_gari_ros2_to_mcu_connect_msg)
{
    return mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_pack_status(system_id, component_id, _status, msg,  gorur_gari_ros2_to_mcu_connect_msg->connected);
}

/**
 * @brief Send a gorur_gari_ros2_to_mcu_connect_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param connected  1 = link established, 0 = link going down
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_send(mavlink_channel_t chan, uint8_t connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN];
    _mav_put_uint8_t(buf, 0, connected);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg, buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
#else
    mavlink_gorur_gari_ros2_to_mcu_connect_msg_t packet;
    packet.connected = connected;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg, (const char *)&packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
#endif
}

/**
 * @brief Send a gorur_gari_ros2_to_mcu_connect_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_send_struct(mavlink_channel_t chan, const mavlink_gorur_gari_ros2_to_mcu_connect_msg_t* gorur_gari_ros2_to_mcu_connect_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_send(chan, gorur_gari_ros2_to_mcu_connect_msg->connected);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg, (const char *)gorur_gari_ros2_to_mcu_connect_msg, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
#endif
}

#if MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, connected);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg, buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
#else
    mavlink_gorur_gari_ros2_to_mcu_connect_msg_t *packet = (mavlink_gorur_gari_ros2_to_mcu_connect_msg_t *)msgbuf;
    packet->connected = connected;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg, (const char *)packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_CRC);
#endif
}
#endif

#endif

// MESSAGE gorur_gari_ros2_to_mcu_connect_msg UNPACKING


/**
 * @brief Get field connected from gorur_gari_ros2_to_mcu_connect_msg message
 *
 * @return  1 = link established, 0 = link going down
 */
static inline uint8_t mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_get_connected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a gorur_gari_ros2_to_mcu_connect_msg message into a struct
 *
 * @param msg The message to decode
 * @param gorur_gari_ros2_to_mcu_connect_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_decode(const mavlink_message_t* msg, mavlink_gorur_gari_ros2_to_mcu_connect_msg_t* gorur_gari_ros2_to_mcu_connect_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gorur_gari_ros2_to_mcu_connect_msg->connected = mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_get_connected(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN? msg->len : MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN;
        memset(gorur_gari_ros2_to_mcu_connect_msg, 0, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg_LEN);
    memcpy(gorur_gari_ros2_to_mcu_connect_msg, _MAV_PAYLOAD(msg), len);
#endif
}
