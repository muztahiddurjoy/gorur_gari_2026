/** @file
 *    @brief MAVLink comm protocol testsuite generated from mcu_to_ros2.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef MCU_TO_ROS2_TESTSUITE_H
#define MCU_TO_ROS2_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_mcu_to_ros2(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_mcu_to_ros2(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_gorur_gari_mcu_to_ros2_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gorur_gari_mcu_to_ros2_msg_t packet_in = {
        963497464,45.0,29,96,163,230,41,108,175,242
    };
    mavlink_gorur_gari_mcu_to_ros2_msg_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.encoder_count = packet_in.encoder_count;
        packet1.heading = packet_in.heading;
        packet1.encoder_speed = packet_in.encoder_speed;
        packet1.encoder_direction = packet_in.encoder_direction;
        packet1.servo = packet_in.servo;
        packet1.sonar_1 = packet_in.sonar_1;
        packet1.sonar_2 = packet_in.sonar_2;
        packet1.sonar_3 = packet_in.sonar_3;
        packet1.sonar_4 = packet_in.sonar_4;
        packet1.button = packet_in.button;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack(system_id, component_id, &msg , packet1.encoder_count , packet1.encoder_speed , packet1.encoder_direction , packet1.servo , packet1.heading , packet1.sonar_1 , packet1.sonar_2 , packet1.sonar_3 , packet1.sonar_4 , packet1.button );
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.encoder_count , packet1.encoder_speed , packet1.encoder_direction , packet1.servo , packet1.heading , packet1.sonar_1 , packet1.sonar_2 , packet1.sonar_3 , packet1.sonar_4 , packet1.button );
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_send(MAVLINK_COMM_1 , packet1.encoder_count , packet1.encoder_speed , packet1.encoder_direction , packet1.servo , packet1.heading , packet1.sonar_1 , packet1.sonar_2 , packet1.sonar_3 , packet1.sonar_4 , packet1.button );
    mavlink_msg_gorur_gari_mcu_to_ros2_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("gorur_gari_mcu_to_ros2_msg") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_gorur_gari_mcu_to_ros2_msg) != NULL);
#endif
}

static void mavlink_test_mcu_to_ros2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_gorur_gari_mcu_to_ros2_msg(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MCU_TO_ROS2_TESTSUITE_H
