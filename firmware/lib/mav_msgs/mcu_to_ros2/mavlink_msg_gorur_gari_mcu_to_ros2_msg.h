#pragma once
// MESSAGE gorur_gari_mcu_to_ros2_msg PACKING

#define MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg 50001


typedef struct __mavlink_gorur_gari_mcu_to_ros2_msg_t {
 int32_t encoder_count; /*<  raw cumulative ticks*/
 float heading; /*<  deg*/
 uint8_t encoder_speed; /*<  speed*/
 uint8_t encoder_direction; /*<  direction*/
 uint8_t servo; /*<  degree*/
 uint8_t sonar_1; /*<  cm*/
 uint8_t sonar_2; /*<  cm*/
 uint8_t sonar_3; /*<  cm*/
 uint8_t sonar_4; /*<  cm*/
 uint8_t button; /*<  0 = released, 1 = pressed*/
} mavlink_gorur_gari_mcu_to_ros2_msg_t;

#define MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN 16
#define MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN 16
#define MAVLINK_MSG_ID_50001_LEN 16
#define MAVLINK_MSG_ID_50001_MIN_LEN 16

#define MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC 157
#define MAVLINK_MSG_ID_50001_CRC 157



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_gorur_gari_mcu_to_ros2_msg { \
    50001, \
    "gorur_gari_mcu_to_ros2_msg", \
    10, \
    {  { "encoder_count", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, encoder_count) }, \
         { "encoder_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, encoder_speed) }, \
         { "encoder_direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, encoder_direction) }, \
         { "servo", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, servo) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, heading) }, \
         { "sonar_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_1) }, \
         { "sonar_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_2) }, \
         { "sonar_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_3) }, \
         { "sonar_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_4) }, \
         { "button", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, button) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_gorur_gari_mcu_to_ros2_msg { \
    "gorur_gari_mcu_to_ros2_msg", \
    10, \
    {  { "encoder_count", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, encoder_count) }, \
         { "encoder_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, encoder_speed) }, \
         { "encoder_direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, encoder_direction) }, \
         { "servo", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, servo) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, heading) }, \
         { "sonar_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_1) }, \
         { "sonar_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_2) }, \
         { "sonar_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_3) }, \
         { "sonar_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, sonar_4) }, \
         { "button", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_gorur_gari_mcu_to_ros2_msg_t, button) }, \
         } \
}
#endif

/**
 * @brief Pack a gorur_gari_mcu_to_ros2_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder_count  raw cumulative ticks
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param heading  deg
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 * @param button  0 = released, 1 = pressed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, float heading, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4, uint8_t button)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN];
    _mav_put_int32_t(buf, 0, encoder_count);
    _mav_put_float(buf, 4, heading);
    _mav_put_uint8_t(buf, 8, encoder_speed);
    _mav_put_uint8_t(buf, 9, encoder_direction);
    _mav_put_uint8_t(buf, 10, servo);
    _mav_put_uint8_t(buf, 11, sonar_1);
    _mav_put_uint8_t(buf, 12, sonar_2);
    _mav_put_uint8_t(buf, 13, sonar_3);
    _mav_put_uint8_t(buf, 14, sonar_4);
    _mav_put_uint8_t(buf, 15, button);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
#else
    mavlink_gorur_gari_mcu_to_ros2_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.heading = heading;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;
    packet.button = button;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
}

/**
 * @brief Pack a gorur_gari_mcu_to_ros2_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder_count  raw cumulative ticks
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param heading  deg
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 * @param button  0 = released, 1 = pressed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, float heading, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4, uint8_t button)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN];
    _mav_put_int32_t(buf, 0, encoder_count);
    _mav_put_float(buf, 4, heading);
    _mav_put_uint8_t(buf, 8, encoder_speed);
    _mav_put_uint8_t(buf, 9, encoder_direction);
    _mav_put_uint8_t(buf, 10, servo);
    _mav_put_uint8_t(buf, 11, sonar_1);
    _mav_put_uint8_t(buf, 12, sonar_2);
    _mav_put_uint8_t(buf, 13, sonar_3);
    _mav_put_uint8_t(buf, 14, sonar_4);
    _mav_put_uint8_t(buf, 15, button);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
#else
    mavlink_gorur_gari_mcu_to_ros2_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.heading = heading;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;
    packet.button = button;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
#endif
}

/**
 * @brief Pack a gorur_gari_mcu_to_ros2_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param encoder_count  raw cumulative ticks
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param heading  deg
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 * @param button  0 = released, 1 = pressed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t encoder_count,uint8_t encoder_speed,uint8_t encoder_direction,uint8_t servo,float heading,uint8_t sonar_1,uint8_t sonar_2,uint8_t sonar_3,uint8_t sonar_4,uint8_t button)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN];
    _mav_put_int32_t(buf, 0, encoder_count);
    _mav_put_float(buf, 4, heading);
    _mav_put_uint8_t(buf, 8, encoder_speed);
    _mav_put_uint8_t(buf, 9, encoder_direction);
    _mav_put_uint8_t(buf, 10, servo);
    _mav_put_uint8_t(buf, 11, sonar_1);
    _mav_put_uint8_t(buf, 12, sonar_2);
    _mav_put_uint8_t(buf, 13, sonar_3);
    _mav_put_uint8_t(buf, 14, sonar_4);
    _mav_put_uint8_t(buf, 15, button);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
#else
    mavlink_gorur_gari_mcu_to_ros2_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.heading = heading;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;
    packet.button = button;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
}

/**
 * @brief Encode a gorur_gari_mcu_to_ros2_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_mcu_to_ros2_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gorur_gari_mcu_to_ros2_msg_t* gorur_gari_mcu_to_ros2_msg)
{
    return mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack(system_id, component_id, msg, gorur_gari_mcu_to_ros2_msg->encoder_count, gorur_gari_mcu_to_ros2_msg->encoder_speed, gorur_gari_mcu_to_ros2_msg->encoder_direction, gorur_gari_mcu_to_ros2_msg->servo, gorur_gari_mcu_to_ros2_msg->heading, gorur_gari_mcu_to_ros2_msg->sonar_1, gorur_gari_mcu_to_ros2_msg->sonar_2, gorur_gari_mcu_to_ros2_msg->sonar_3, gorur_gari_mcu_to_ros2_msg->sonar_4, gorur_gari_mcu_to_ros2_msg->button);
}

/**
 * @brief Encode a gorur_gari_mcu_to_ros2_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_mcu_to_ros2_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gorur_gari_mcu_to_ros2_msg_t* gorur_gari_mcu_to_ros2_msg)
{
    return mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack_chan(system_id, component_id, chan, msg, gorur_gari_mcu_to_ros2_msg->encoder_count, gorur_gari_mcu_to_ros2_msg->encoder_speed, gorur_gari_mcu_to_ros2_msg->encoder_direction, gorur_gari_mcu_to_ros2_msg->servo, gorur_gari_mcu_to_ros2_msg->heading, gorur_gari_mcu_to_ros2_msg->sonar_1, gorur_gari_mcu_to_ros2_msg->sonar_2, gorur_gari_mcu_to_ros2_msg->sonar_3, gorur_gari_mcu_to_ros2_msg->sonar_4, gorur_gari_mcu_to_ros2_msg->button);
}

/**
 * @brief Encode a gorur_gari_mcu_to_ros2_msg struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gorur_gari_mcu_to_ros2_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gorur_gari_mcu_to_ros2_msg_t* gorur_gari_mcu_to_ros2_msg)
{
    return mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack_status(system_id, component_id, _status, msg,  gorur_gari_mcu_to_ros2_msg->encoder_count, gorur_gari_mcu_to_ros2_msg->encoder_speed, gorur_gari_mcu_to_ros2_msg->encoder_direction, gorur_gari_mcu_to_ros2_msg->servo, gorur_gari_mcu_to_ros2_msg->heading, gorur_gari_mcu_to_ros2_msg->sonar_1, gorur_gari_mcu_to_ros2_msg->sonar_2, gorur_gari_mcu_to_ros2_msg->sonar_3, gorur_gari_mcu_to_ros2_msg->sonar_4, gorur_gari_mcu_to_ros2_msg->button);
}

/**
 * @brief Send a gorur_gari_mcu_to_ros2_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param encoder_count  raw cumulative ticks
 * @param encoder_speed  speed
 * @param encoder_direction  direction
 * @param servo  degree
 * @param heading  deg
 * @param sonar_1  cm
 * @param sonar_2  cm
 * @param sonar_3  cm
 * @param sonar_4  cm
 * @param button  0 = released, 1 = pressed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gorur_gari_mcu_to_ros2_msg_send(mavlink_channel_t chan, int32_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, float heading, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4, uint8_t button)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN];
    _mav_put_int32_t(buf, 0, encoder_count);
    _mav_put_float(buf, 4, heading);
    _mav_put_uint8_t(buf, 8, encoder_speed);
    _mav_put_uint8_t(buf, 9, encoder_direction);
    _mav_put_uint8_t(buf, 10, servo);
    _mav_put_uint8_t(buf, 11, sonar_1);
    _mav_put_uint8_t(buf, 12, sonar_2);
    _mav_put_uint8_t(buf, 13, sonar_3);
    _mav_put_uint8_t(buf, 14, sonar_4);
    _mav_put_uint8_t(buf, 15, button);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg, buf, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
#else
    mavlink_gorur_gari_mcu_to_ros2_msg_t packet;
    packet.encoder_count = encoder_count;
    packet.heading = heading;
    packet.encoder_speed = encoder_speed;
    packet.encoder_direction = encoder_direction;
    packet.servo = servo;
    packet.sonar_1 = sonar_1;
    packet.sonar_2 = sonar_2;
    packet.sonar_3 = sonar_3;
    packet.sonar_4 = sonar_4;
    packet.button = button;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg, (const char *)&packet, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
#endif
}

/**
 * @brief Send a gorur_gari_mcu_to_ros2_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gorur_gari_mcu_to_ros2_msg_send_struct(mavlink_channel_t chan, const mavlink_gorur_gari_mcu_to_ros2_msg_t* gorur_gari_mcu_to_ros2_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_send(chan, gorur_gari_mcu_to_ros2_msg->encoder_count, gorur_gari_mcu_to_ros2_msg->encoder_speed, gorur_gari_mcu_to_ros2_msg->encoder_direction, gorur_gari_mcu_to_ros2_msg->servo, gorur_gari_mcu_to_ros2_msg->heading, gorur_gari_mcu_to_ros2_msg->sonar_1, gorur_gari_mcu_to_ros2_msg->sonar_2, gorur_gari_mcu_to_ros2_msg->sonar_3, gorur_gari_mcu_to_ros2_msg->sonar_4, gorur_gari_mcu_to_ros2_msg->button);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg, (const char *)gorur_gari_mcu_to_ros2_msg, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
#endif
}

#if MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gorur_gari_mcu_to_ros2_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t encoder_count, uint8_t encoder_speed, uint8_t encoder_direction, uint8_t servo, float heading, uint8_t sonar_1, uint8_t sonar_2, uint8_t sonar_3, uint8_t sonar_4, uint8_t button)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, encoder_count);
    _mav_put_float(buf, 4, heading);
    _mav_put_uint8_t(buf, 8, encoder_speed);
    _mav_put_uint8_t(buf, 9, encoder_direction);
    _mav_put_uint8_t(buf, 10, servo);
    _mav_put_uint8_t(buf, 11, sonar_1);
    _mav_put_uint8_t(buf, 12, sonar_2);
    _mav_put_uint8_t(buf, 13, sonar_3);
    _mav_put_uint8_t(buf, 14, sonar_4);
    _mav_put_uint8_t(buf, 15, button);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg, buf, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
#else
    mavlink_gorur_gari_mcu_to_ros2_msg_t *packet = (mavlink_gorur_gari_mcu_to_ros2_msg_t *)msgbuf;
    packet->encoder_count = encoder_count;
    packet->heading = heading;
    packet->encoder_speed = encoder_speed;
    packet->encoder_direction = encoder_direction;
    packet->servo = servo;
    packet->sonar_1 = sonar_1;
    packet->sonar_2 = sonar_2;
    packet->sonar_3 = sonar_3;
    packet->sonar_4 = sonar_4;
    packet->button = button;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg, (const char *)packet, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_CRC);
#endif
}
#endif

#endif

// MESSAGE gorur_gari_mcu_to_ros2_msg UNPACKING


/**
 * @brief Get field encoder_count from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  raw cumulative ticks
 */
static inline int32_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_encoder_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field encoder_speed from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  speed
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_encoder_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field encoder_direction from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  direction
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_encoder_direction(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field servo from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  degree
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_servo(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field heading from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  deg
 */
static inline float mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field sonar_1 from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field sonar_2 from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field sonar_3 from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field sonar_4 from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  cm
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field button from gorur_gari_mcu_to_ros2_msg message
 *
 * @return  0 = released, 1 = pressed
 */
static inline uint8_t mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_button(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Decode a gorur_gari_mcu_to_ros2_msg message into a struct
 *
 * @param msg The message to decode
 * @param gorur_gari_mcu_to_ros2_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_gorur_gari_mcu_to_ros2_msg_decode(const mavlink_message_t* msg, mavlink_gorur_gari_mcu_to_ros2_msg_t* gorur_gari_mcu_to_ros2_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gorur_gari_mcu_to_ros2_msg->encoder_count = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_encoder_count(msg);
    gorur_gari_mcu_to_ros2_msg->heading = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_heading(msg);
    gorur_gari_mcu_to_ros2_msg->encoder_speed = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_encoder_speed(msg);
    gorur_gari_mcu_to_ros2_msg->encoder_direction = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_encoder_direction(msg);
    gorur_gari_mcu_to_ros2_msg->servo = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_servo(msg);
    gorur_gari_mcu_to_ros2_msg->sonar_1 = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_1(msg);
    gorur_gari_mcu_to_ros2_msg->sonar_2 = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_2(msg);
    gorur_gari_mcu_to_ros2_msg->sonar_3 = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_3(msg);
    gorur_gari_mcu_to_ros2_msg->sonar_4 = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_sonar_4(msg);
    gorur_gari_mcu_to_ros2_msg->button = mavlink_msg_gorur_gari_mcu_to_ros2_msg_get_button(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN? msg->len : MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN;
        memset(gorur_gari_mcu_to_ros2_msg, 0, MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_LEN);
    memcpy(gorur_gari_mcu_to_ros2_msg, _MAV_PAYLOAD(msg), len);
#endif
}
