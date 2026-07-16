/** @file
 *  @brief MAVLink comm protocol generated from serial_msg.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_SERIAL_MSG_H
#define MAVLINK_SERIAL_MSG_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_SERIAL_MSG.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_SERIAL_MSG_XML_HASH 5751880744690870182

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{50001, 39, 8, 8, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_SERIAL_MSG

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
#include "./mavlink_msg_gorur_gari_serial_msg.h"

// base include



#if MAVLINK_SERIAL_MSG_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_gorur_gari_serial_msg}
# define MAVLINK_MESSAGE_NAMES {{ "gorur_gari_serial_msg", 50001 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_SERIAL_MSG_H
