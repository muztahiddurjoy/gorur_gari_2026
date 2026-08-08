/** @file
 *  @brief MAVLink comm protocol generated from ros2_to_mcu.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_ROS2_TO_MCU_H
#define MAVLINK_ROS2_TO_MCU_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_ROS2_TO_MCU.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_ROS2_TO_MCU_XML_HASH 1858230744608499424

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{50002, 194, 2, 2, 0, 0, 0}, {50003, 149, 1, 1, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_ROS2_TO_MCU

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_gorur_gari_ros2_to_mcu_msg.h"
#include "./mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg.h"

// base include



#if MAVLINK_ROS2_TO_MCU_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_gorur_gari_ros2_to_mcu_msg, MAVLINK_MESSAGE_INFO_gorur_gari_ros2_to_mcu_connect_msg}
# define MAVLINK_MESSAGE_NAMES {{ "gorur_gari_ros2_to_mcu_connect_msg", 50003 }, { "gorur_gari_ros2_to_mcu_msg", 50002 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_ROS2_TO_MCU_H
