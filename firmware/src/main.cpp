#include <Arduino.h>
#include "steering_control.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "pins.h"
#include "config.h"
#include <MAVLink.h>

#ifdef ENABLE_RC_CAR
#include "rc_car.h"
#endif

SteeringController steer(STEERING_SERVO_PIN);
MotorController motor(PWM_THROTTLE_PIN, IN_A, IN_B, STANDBY_PIN);
EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTS_PER_REV);

#ifdef ENABLE_RC_CAR
RcCar rc(steer, motor, encoder);
#endif

const uint8_t system_id = 1;
const uint8_t component_id = 1;


void setup(){
    Serial.begin(115200);
    steer.begin(SERVO_FREQUENCY_HZ);
    steer.to(STEERING_CENTER_ANGLE);
    motor.begin(MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    encoder.begin(ENCODER_SAMPLE_INTERVAL_MS);

#ifdef ENABLE_RC_CAR
    rc.begin();
#endif
}

void loop(){
    encoder.update();

#ifdef ENABLE_RC_CAR
    rc.update();
#endif

    // mavlink_message_t msg;
    // uint8_t buf[MAVLINK_MAX_PACKET_LEN];


    // mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED , 0, MAV_STATE_ACTIVE);
    // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Serial.write(buf, len);
    // while(Serial.available()>0){
    //     uint8_t c = Serial.read();
    //     mavlink_message_t rx_msg;
    //     mavlink_status_t status;

    //     if(mavlink_parse_char(MAVLINK_COMM_0, c, &rx_msg, &status)){
    //         if(rx_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
    //             mavlink_heartbeat_t heartbeat;
    //             mavlink_msg_heartbeat_decode(&rx_msg, &heartbeat);
    //             Serial.print("Received heartbeat: ");
    //             Serial.print("Type: "); Serial.print(heartbeat.type);
    //             Serial.print(", Autopilot: "); Serial.print(heartbeat.autopilot);
    //             Serial.print(", Base Mode: "); Serial.print(heartbeat.base_mode);
    //             Serial.print(", Custom Mode: "); Serial.print(heartbeat.custom_mode);
    //             Serial.print(", System Status: "); Serial.println(heartbeat.system_status);
    //         }
    //     }
    // }
    // steer.to(90+50);
    // delay(1000);
    // steer.to(90-55);
    // delay(1000);
}
