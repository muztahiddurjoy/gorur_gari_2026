#pragma once
// MESSAGE gorur_gari_serial_msg PACKING

#define MAVLINK_MSG_ID_gorur_gari_serial_msg 50001


typedef struct __mavlink_gorur_gari_serial_msg_t {
 uint8_t encoder_count; /*<  count*/
 uint8_t encoder_speed; /*<  speed*/
 uint8_t encoder_direction; /*<  direction*/
 uint8_t servo; /*<  degree*/
 uint8_t sonar_1; /*<  cm*/
 uint8_t sonar_2; /*<  cm*/
 uint8_t sonar_3; /*<  cm*/
 uint8_t sonar_4; /*<  cm*/
} mavlink_gorur_gari_serial_msg_t;

#define MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN 8
#define MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN 8
#define MAVLINK_MSG_ID_50001_LEN 8
#define MAVLINK_MSG_ID_50001_MIN_LEN 8

#define MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC 39
#define MAVLINK_MSG_ID_50001_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_gorur_gari_serial_msg { \
    50001, \
    "gorur_gari_serial_msg", \
    8, \
    {  { "encoder_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gorur_gari_serial_msg_t, encoder_count) }, \
         { "encoder_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gorur_gari_serial_msg_t, encoder_speed) }, \
         { "encoder_direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gorur_gari_serial_msg_t, encoder_direction) }, \
         { "servo", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gorur_gari_serial_msg_t, servo) }, \
         { "sonar_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_1) }, \
         { "sonar_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_2) }, \
         { "sonar_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_3) }, \
         { "sonar_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_gorur_gari_serial_msg { \
    "gorur_gari_serial_msg", \
    8, \
    {  { "encoder_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gorur_gari_serial_msg_t, encoder_count) }, \
         { "encoder_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gorur_gari_serial_msg_t, encoder_speed) }, \
         { "encoder_direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gorur_gari_serial_msg_t, encoder_direction) }, \
         { "servo", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gorur_gari_serial_msg_t, servo) }, \
         { "sonar_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_1) }, \
         { "sonar_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_2) }, \
         { "sonar_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_3) }, \
         { "sonar_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_gorur_gari_serial_msg_t, sonar_4) }, \
         } \
}
#endif

/**
 * @brief Pack a gorur_gari_serial_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder_count  count
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_serial_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN];
    _mav_put_uint8_t(buf, 0, encoder_count);
    _mav_put_uint8_t(buf, 1, encoder_speed);
    _mav_put_uint8_t(buf, 2, encoder_direction);
    _mav_put_uint8_t(buf, 3, servo);
    _mav_put_uint8_t(buf, 4, sonar_1);
    _mav_put_uint8_t(buf, 5, sonar_2);
    _mav_put_uint8_t(buf, 6, sonar_3);
    _mav_put_uint8_t(buf, 7, sonar_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
#else
    mavlink_gorur_gari_serial_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_serial_msg;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
}

/**
 * @brief Pack a gorur_gari_serial_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder_count  count
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_serial_msg_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN];
    _mav_put_uint8_t(buf, 0, encoder_count);
    _mav_put_uint8_t(buf, 1, encoder_speed);
    _mav_put_uint8_t(buf, 2, encoder_direction);
    _mav_put_uint8_t(buf, 3, servo);
    _mav_put_uint8_t(buf, 4, sonar_1);
    _mav_put_uint8_t(buf, 5, sonar_2);
    _mav_put_uint8_t(buf, 6, sonar_3);
    _mav_put_uint8_t(buf, 7, sonar_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
#else
    mavlink_gorur_gari_serial_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_serial_msg;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
#endif
}

/**
 * @brief Pack a gorur_gari_serial_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param encoder_count  count
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_serial_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t encoder_count,uint8_t encoder_speed,uint8_t encoder_direction,uint8_t servo,uint8_t sonar_1,uint8_t sonar_2,uint8_t sonar_3,uint8_t sonar_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN];
    _mav_put_uint8_t(buf, 0, encoder_count);
    _mav_put_uint8_t(buf, 1, encoder_speed);
    _mav_put_uint8_t(buf, 2, encoder_direction);
    _mav_put_uint8_t(buf, 3, servo);
    _mav_put_uint8_t(buf, 4, sonar_1);
    _mav_put_uint8_t(buf, 5, sonar_2);
    _mav_put_uint8_t(buf, 6, sonar_3);
    _mav_put_uint8_t(buf, 7, sonar_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
#else
    mavlink_gorur_gari_serial_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_serial_msg;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
}

/**
 * @brief Encode a gorur_gari_serial_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_serial_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_serial_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gorur_gari_serial_msg_t* gorur_gari_serial_msg)
{
    return mavlink_msg_gorur_gari_serial_msg_pack(system_id, component_id, msg, gorur_gari_serial_msg->encoder_count, gorur_gari_serial_msg->encoder_speed, gorur_gari_serial_msg->encoder_direction, gorur_gari_serial_msg->servo, gorur_gari_serial_msg->sonar_1, gorur_gari_serial_msg->sonar_2, gorur_gari_serial_msg->sonar_3, gorur_gari_serial_msg->sonar_4);
}

/**
 * @brief Encode a gorur_gari_serial_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_serial_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_serial_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gorur_gari_serial_msg_t* gorur_gari_serial_msg)
{
    return mavlink_msg_gorur_gari_serial_msg_pack_chan(system_id, component_id, chan, msg, gorur_gari_serial_msg->encoder_count, gorur_gari_serial_msg->encoder_speed, gorur_gari_serial_msg->encoder_direction, gorur_gari_serial_msg->servo, gorur_gari_serial_msg->sonar_1, gorur_gari_serial_msg->sonar_2, gorur_gari_serial_msg->sonar_3, gorur_gari_serial_msg->sonar_4);
}

/**
 * @brief Encode a gorur_gari_serial_msg struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_serial_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_serial_msg_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gorur_gari_serial_msg_t* gorur_gari_serial_msg)
{
    return mavlink_msg_gorur_gari_serial_msg_pack_status(system_id, component_id, _status, msg,  gorur_gari_serial_msg->encoder_count, gorur_gari_serial_msg->encoder_speed, gorur_gari_serial_msg->encoder_direction, gorur_gari_serial_msg->servo, gorur_gari_serial_msg->sonar_1, gorur_gari_serial_msg->sonar_2, gorur_gari_serial_msg->sonar_3, gorur_gari_serial_msg->sonar_4);
}

/**
 * @brief Send a gorur_gari_serial_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param encoder_count  count
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gorur_gari_serial_msg_send(mavlink_channel_t chan, uint8_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN];
    _mav_put_uint8_t(buf, 0, encoder_count);
    _mav_put_uint8_t(buf, 1, encoder_speed);
    _mav_put_uint8_t(buf, 2, encoder_direction);
    _mav_put_uint8_t(buf, 3, servo);
    _mav_put_uint8_t(buf, 4, sonar_1);
    _mav_put_uint8_t(buf, 5, sonar_2);
    _mav_put_uint8_t(buf, 6, sonar_3);
    _mav_put_uint8_t(buf, 7, sonar_4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_serial_msg, buf, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
#else
    mavlink_gorur_gari_serial_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_serial_msg, (const char *)&packet, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
#endif
}

/**
 * @brief Send a gorur_gari_serial_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gorur_gari_serial_msg_send_struct(mavlink_channel_t chan, const mavlink_gorur_gari_serial_msg_t* gorur_gari_serial_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gorur_gari_serial_msg_send(chan, gorur_gari_serial_msg->encoder_count, gorur_gari_serial_msg->encoder_speed, gorur_gari_serial_msg->encoder_direction, gorur_gari_serial_msg->servo, gorur_gari_serial_msg->sonar_1, gorur_gari_serial_msg->sonar_2, gorur_gari_serial_msg->sonar_3, gorur_gari_serial_msg->sonar_4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_serial_msg, (const char *)gorur_gari_serial_msg, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
#endif
}

#if MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gorur_gari_serial_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, encoder_count);
    _mav_put_uint8_t(buf, 1, encoder_speed);
    _mav_put_uint8_t(buf, 2, encoder_direction);
    _mav_put_uint8_t(buf, 3, servo);
    _mav_put_uint8_t(buf, 4, sonar_1);
    _mav_put_uint8_t(buf, 5, sonar_2);
    _mav_put_uint8_t(buf, 6, sonar_3);
    _mav_put_uint8_t(buf, 7, sonar_4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_serial_msg, buf, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
#else
    mavlink_gorur_gari_serial_msg_t *packet = (mavlink_gorur_gari_serial_msg_t *)msgbuf;
    packet->encoder_count = encoder_count;
    packet->encoder_speed = encoder_speed;
    packet->encoder_direction = encoder_direction;
    packet->servo = servo;
    packet->sonar_1 = sonar_1;
    packet->sonar_2 = sonar_2;
    packet->sonar_3 = sonar_3;
    packet->sonar_4 = sonar_4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_serial_msg, (const char *)packet, MAVLINK_MSG_ID_gorur_gari_serial_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN, MAVLINK_MSG_ID_gorur_gari_serial_msg_CRC);
#endif
}
#endif

#endif

// MESSAGE gorur_gari_serial_msg UNPACKING


/**
 * @brief Get field encoder_count from gorur_gari_serial_msg message
 *
 * @return  count
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_encoder_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field encoder_speed from gorur_gari_serial_msg message
 *
 * @return  speed
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_encoder_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field encoder_direction from gorur_gari_serial_msg message
 *
 * @return  direction
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_encoder_direction(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field servo from gorur_gari_serial_msg message
 *
 * @return  degree
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_servo(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field sonar_1 from gorur_gari_serial_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_sonar_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sonar_2 from gorur_gari_serial_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_sonar_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field sonar_3 from gorur_gari_serial_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_sonar_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field sonar_4 from gorur_gari_serial_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_serial_msg_get_sonar_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a gorur_gari_serial_msg message into a struct
 *
 * @param msg The message to decode
 * @param gorur_gari_serial_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_gorur_gari_serial_msg_decode(const mavlink_message_t* msg, mavlink_gorur_gari_serial_msg_t* gorur_gari_serial_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gorur_gari_serial_msg->encoder_count = mavlink_msg_gorur_gari_serial_msg_get_encoder_count(msg);
    gorur_gari_serial_msg->encoder_speed = mavlink_msg_gorur_gari_serial_msg_get_encoder_speed(msg);
    gorur_gari_serial_msg->encoder_direction = mavlink_msg_gorur_gari_serial_msg_get_encoder_direction(msg);
    gorur_gari_serial_msg->servo = mavlink_msg_gorur_gari_serial_msg_get_servo(msg);
    gorur_gari_serial_msg->sonar_1 = mavlink_msg_gorur_gari_serial_msg_get_sonar_1(msg);
    gorur_gari_serial_msg->sonar_2 = mavlink_msg_gorur_gari_serial_msg_get_sonar_2(msg);
    gorur_gari_serial_msg->sonar_3 = mavlink_msg_gorur_gari_serial_msg_get_sonar_3(msg);
    gorur_gari_serial_msg->sonar_4 = mavlink_msg_gorur_gari_serial_msg_get_sonar_4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN? msg->len : MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN;
        memset(gorur_gari_serial_msg, 0, MAVLINK_MSG_ID_gorur_gari_serial_msg_LEN);
    memcpy(gorur_gari_serial_msg, _MAV_PAYLOAD(msg), len);
#endif
}
