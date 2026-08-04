#include <Arduino.h>
#include "steering_control.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "Wire.h"
#include "display_control.h"
// #include "heading.h"
#include "pins.h"
#include "config.h"

#include "ros2_to_mcu/mavlink.h"
#include "mcu_to_ros2/mavlink.h"



SteeringController steer(STEERING_SERVO_PIN);
MotorController motor(PWM_THROTTLE_PIN, IN_A, IN_B, STANDBY_PIN);
EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTS_PER_REV);
DisplayController display;
// Heading heading;
const uint8_t system_id = 1;
const uint8_t component_id = 200;


void setup(){
    Serial.begin(115200);
    steer.begin(SERVO_FREQUENCY_HZ);
    steer.to(STEERING_CENTER_ANGLE);
    motor.begin(MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    encoder.begin(ENCODER_SAMPLE_INTERVAL_MS);
    Wire.begin(I2C_SDA, I2C_SCL);
    display.begin();
    // heading.begin();

    
}

void loop(){
     encoder.update();
    // heading.update();
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    

    display.displayText("Speed: " + String(motor.speed()), 0, 0, 1);
    display.displayText("Encoder: " + String(encoder.count()), 0, 10, 1);
    display.displayText("Steering: " + String(steer.getAngle()), 0, 20, 1);
    // display.displayText("Yaw: " + String(heading.getHeading()), 0, 30, 1);

    display.updateScreen();

    
    if(Serial.available()>0){
        uint8_t c = Serial.read();
        
        mavlink_message_t received_msg;
        mavlink_status_t status;

        if(mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &status)){
            if(received_msg.msgid == MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_msg){
                int throttle = mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_throttle(&received_msg);
                motor.to(throttle);
                uint8_t steering = mavlink_msg_gorur_gari_ros2_to_mcu_msg_get_steering(&received_msg);
                steer.to(steering);
            }
        }
    }


}



