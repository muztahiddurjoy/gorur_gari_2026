#include <Arduino.h>
#include "steering_control.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "Wire.h"
#include "display_control.h"
#include "heading.h"
#include "pins.h"
#include "config.h"
#include "shared_data.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ros2_to_mcu/mavlink.h"
#include "mcu_to_ros2/mavlink.h"



SteeringController steer(STEERING_SERVO_PIN);
MotorController motor(PWM_THROTTLE_PIN, IN_A, IN_B, STANDBY_PIN);
EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTS_PER_REV);
// Heading heading;
const uint8_t system_id = 1;
const uint8_t component_id = 200;



// Global I2C task handle (optional)
TaskHandle_t i2cTaskHandle = nullptr;

void i2cTask(void *pvParameters) {
    // Create display and IMU objects locally – they own the I2C bus
    DisplayController display;
    if (!display.begin()) {
        Serial.println("Display init failed");
    }

    Heading heading;
    if (!heading.begin()) {
        Serial.println("IMU init failed");
    }

    TickType_t lastDisplayUpdate = xTaskGetTickCount();
    const TickType_t displayInterval = pdMS_TO_TICKS(50);   // 50 ms refresh
    const TickType_t imuInterval = pdMS_TO_TICKS(10);        // IMU read every 10 ms

    TickType_t lastIMUUpdate = xTaskGetTickCount();

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // --- Update IMU (every 10 ms) ---
        if (now - lastIMUUpdate >= imuInterval) {
            lastIMUUpdate = now;
            heading.update();
            float yaw = heading.getHeading();

            // Write heading to shared struct
            if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
                shared.heading = yaw;
                xSemaphoreGive(sharedMutex);
            }
        }

        // --- Update Display (every 50 ms) ---
        if (now - lastDisplayUpdate >= displayInterval) {
            lastDisplayUpdate = now;

            // Read display strings from shared struct
            String speed, encoder, steering;
            if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
                speed = shared.speedText;
                encoder = shared.encoderText;
                steering = shared.steeringText;
                // Optionally read heading to display as well:
                // String yawText = "Yaw: " + String(shared.heading);
                xSemaphoreGive(sharedMutex);
            }

            // Draw everything
            display.clear();
            display.displayText(speed, 0, 0, 1);
            display.displayText(encoder, 0, 10, 1);
            display.displayText(steering, 0, 20, 1);
            display.displayText("Yaw: " + String(shared.heading), 0, 30, 1);
            display.updateScreen();    // single I2C transfer
        }

        vTaskDelay(1);  // Yield to other tasks (small delay)
    }
}


void setup(){
    Serial.begin(115200);
    steer.begin(SERVO_FREQUENCY_HZ);
    steer.to(STEERING_CENTER_ANGLE);
    motor.begin(MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    encoder.begin(ENCODER_SAMPLE_INTERVAL_MS);
    Wire.begin(I2C_SDA, I2C_SCL);
    
    sharedMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(i2cTask,"I2C Task", 8192, NULL, 1, &i2cTaskHandle, 0);
}

void loop(){
     encoder.update();

     if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
        shared.speedText = "Speed: " + String(motor.speed());
        shared.encoderText = "Encoder: " + String(encoder.count());
        shared.steeringText = "Steering: " + String(steer.getAngle());
        // shared.heading is written by the I2C task, we just read it if needed
        xSemaphoreGive(sharedMutex);
    }


    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    

    
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



