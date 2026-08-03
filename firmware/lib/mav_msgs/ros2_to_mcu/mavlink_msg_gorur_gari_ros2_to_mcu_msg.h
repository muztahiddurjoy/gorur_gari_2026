#pragma once
// MESSAGE gorur_gari_ros2_to_mcu_msg PACKING

#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg 50002


typedef struct __mavlink_gorur_gari_ros2_to_mcu_msg_t {
 int8_t throttle; /*<  throttle*/
 uint8_t steering; /*<  steering*/
} mavlink_gorur_gari_ros2_to_mcu_msg_t;

#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN 2
#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN 2
#define MAVLINK_MSG_ID_50002_LEN 2
#define MAVLINK_MSG_ID_50002_MIN_LEN 2

#define MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC 194
#define MAVLINK_MSG_ID_50002_CRC 194



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_gorur_gari_ros2_to_mcu_msg { \
    50002, \
    "gorur_gari_ros2_to_mcu_msg", \
    2, \
    {  { "throttle", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_gorur_gari_ros2_to_mcu_msg_t, throttle) }, \
         { "steering", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gorur_gari_ros2_to_mcu_msg_t, steering) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_gorur_gari_ros2_to_mcu_msg { \
    "gorur_gari_ros2_to_mcu_msg", \
    2, \
    {  { "throttle", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_gorur_gari_ros2_to_mcu_msg_t, throttle) }, \
         { "steering", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gorur_gari_ros2_to_mcu_msg_t, steering) }, \
         } \
}
#endif

/**
 * @brief Pack a gorur_gari_ros2_to_mcu_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param throttle  throttle
 * @param steering  steering
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t throttle, uint8_t steering)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN];
    _mav_put_int8_t(buf, 0, throttle);
    _mav_put_uint8_t(buf, 1, steering);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
#else
    mavlink_gorur_gari_ros2_to_mcu_msg_t packet;
    packet.throttle = throttle;
    packet.steering = steering;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
}

/**
 * @brief Pack a gorur_gari_ros2_to_mcu_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param throttle  throttle
 * @param steering  steering
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int8_t throttle, uint8_t steering)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN];
    _mav_put_int8_t(buf, 0, throttle);
    _mav_put_uint8_t(buf, 1, steering);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
#else
    mavlink_gorur_gari_ros2_to_mcu_msg_t packet;
    packet.throttle = throttle;
    packet.steering = steering;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
#endif
}

/**
 * @brief Pack a gorur_gari_ros2_to_mcu_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param throttle  throttle
 * @param steering  steering
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t throttle,uint8_t steering)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN];
    _mav_put_int8_t(buf, 0, throttle);
    _mav_put_uint8_t(buf, 1, steering);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
#else
    mavlink_gorur_gari_ros2_to_mcu_msg_t packet;
    packet.throttle = throttle;
    packet.steering = steering;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
}

/**
 * @brief Encode a gorur_gari_ros2_to_mcu_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_ros2_to_mcu_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gorur_gari_ros2_to_mcu_msg_t* gorur_gari_ros2_to_mcu_msg)
{
    return mavlink_msg_gorur_gari_ros2_to_mcu_msg_pack(system_id, component_id, msg, gorur_gari_ros2_to_mcu_msg->throttle, gorur_gari_ros2_to_mcu_msg->steering);
}

/**
 * @brief Encode a gorur_gari_ros2_to_mcu_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_ros2_to_mcu_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gorur_gari_ros2_to_mcu_msg_t* gorur_gari_ros2_to_mcu_msg)
{
    return mavlink_msg_gorur_gari_ros2_to_mcu_msg_pack_chan(system_id, component_id, chan, msg, gorur_gari_ros2_to_mcu_msg->throttle, gorur_gari_ros2_to_mcu_msg->steering);
}

/**
 * @brief Encode a gorur_gari_ros2_to_mcu_msg struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_ros2_to_mcu_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gorur_gari_ros2_to_mcu_msg_t* gorur_gari_ros2_to_mcu_msg)
{
    return mavlink_msg_gorur_gari_ros2_to_mcu_msg_pack_status(system_id, component_id, _status, msg,  gorur_gari_ros2_to_mcu_msg->throttle, gorur_gari_ros2_to_mcu_msg->steering);
}

/**
 * @brief Send a gorur_gari_ros2_to_mcu_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param throttle  throttle
 * @param steering  steering
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gorur_gari_ros2_to_mcu_msg_send(mavlink_channel_t chan, int8_t throttle, uint8_t steering)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN];
    _mav_put_int8_t(buf, 0, throttle);
    _mav_put_uint8_t(buf, 1, steering);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg, buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
#else
    mavlink_gorur_gari_ros2_to_mcu_msg_t packet;
    packet.throttle = throttle;
    packet.steering = steering;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg, (const char *)&packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
#endif
}

/**
 * @brief Send a gorur_gari_ros2_to_mcu_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gorur_gari_ros2_to_mcu_msg_send_struct(mavlink_channel_t chan, const mavlink_gorur_gari_ros2_to_mcu_msg_t* gorur_gari_ros2_to_mcu_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gorur_gari_ros2_to_mcu_msg_send(chan, gorur_gari_ros2_to_mcu_msg->throttle, gorur_gari_ros2_to_mcu_msg->steering);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg, (const char *)gorur_gari_ros2_to_mcu_msg, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
#endif
}

#if MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gorur_gari_ros2_to_mcu_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t throttle, uint8_t steering)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int8_t(buf, 0, throttle);
    _mav_put_uint8_t(buf, 1, steering);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg, buf, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
#else
    mavlink_gorur_gari_ros2_to_mcu_msg_t *packet = (mavlink_gorur_gari_ros2_to_mcu_msg_t *)msgbuf;
    packet->throttle = throttle;
    packet->steering = steering;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg, (const char *)packet, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_CRC);
#endif
}
#endif

#endif

// MESSAGE gorur_gari_ros2_to_mcu_msg UNPACKING


/**
 * @brief Get field throttle from gorur_gari_ros2_to_mcu_msg message
 *
 * @return  throttle
 */
static inline int8_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  0);
}

/**
 * @brief Get field steering from gorur_gari_ros2_to_mcu_msg message
 *
 * @return  steering
 */
static inline uint8_t mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_steering(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a gorur_gari_ros2_to_mcu_msg message into a struct
 *
 * @param msg The message to decode
 * @param gorur_gari_ros2_to_mcu_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_gorur_gari_ros2_to_mcu_msg_decode(const mavlink_message_t* msg, mavlink_gorur_gari_ros2_to_mcu_msg_t* gorur_gari_ros2_to_mcu_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gorur_gari_ros2_to_mcu_msg->throttle = mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_throttle(msg);
    gorur_gari_ros2_to_mcu_msg->steering = mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_steering(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN? msg->len : MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN;
        memset(gorur_gari_ros2_to_mcu_msg, 0, MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg_LEN);
    memcpy(gorur_gari_ros2_to_mcu_msg, _MAV_PAYLOAD(msg), len);
#endif
}
