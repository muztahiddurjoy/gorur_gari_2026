#include <Arduino.h>
#include "steering_control.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "pins.h"
#include "config.h"
#include "serial_msg/mavlink.h"



SteeringController steer(STEERING_SERVO_PIN);
MotorController motor(PWM_THROTTLE_PIN, IN_A, IN_B, STANDBY_PIN);
EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTS_PER_REV);

const uint8_t system_id = 1;
const uint8_t component_id = 200;


void setup(){
    Serial.begin(115200);
    steer.begin(SERVO_FREQUENCY_HZ);
    steer.to(STEERING_CENTER_ANGLE);
    motor.begin(MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    encoder.begin(ENCODER_SAMPLE_INTERVAL_MS);

}

void loop(){
    encoder.update();
    mavlink_mesage_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t current_speed = motor.speed();
    uint8_t current_encoder_count = encoder.count();
    uint8_t servo_angle = steer.getAngle();
    uint8_t d1 = 0;
    uint8_t d2 = 0;
    uint8_t d3 = 0;

    mavlink_msg_gorur_gari_serial_msg_pack(system_id, component_id, &msg, curren_speed, current_encoder_count, servo_angle, d1, d2, d3);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);

    if(Serial.available()>0){
        uint8_t c = Serial.read();
        
        mavlink_message_t received_msg;
        mavlink_status_t status;

        if(mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &status)){
            if(received_msg.msgid == MAVLINK_MSG_ID_gorur_gari_serial_msg){
                uint8_t speed = mavlink_msg_gorur_gari_serial_msg_get_encoder_speed(&received_msg);
                motor.to(speed);
                uint8_t angle = mavlink_msg_gorur_gari_serial_msg_get_servo(&received_msg);
                steer.to(angle);
            }
        }
    }


}
