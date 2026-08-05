#include <Arduino.h>
#include "steering_control.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "sonar_reader.h"
#include "Wire.h"
#include "display_control.h"
#include "heading.h"
#include "pins.h"
#include "config.h"
#include "debug_serial.h"
#include "shared_data.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ros2_to_mcu/mavlink.h"
// both dialects generate a header guarded by the same MAVLINK_H macro, since
// each one is a self-contained mavlink.h. undef it so the second include
// isn't skipped and gorur_gari_mcu_to_ros2_msg actually gets declared.
#undef MAVLINK_H
#include "mcu_to_ros2/mavlink.h"



SteeringController steer(STEERING_SERVO_PIN);
MotorController motor(PWM_THROTTLE_PIN, IN_A, IN_B, STANDBY_PIN);
EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTS_PER_REV);
SonarReader sonarFront(SONAR_FRONT_TRIG_PIN, SONAR_FRONT_ECHO_PIN, SONAR_ECHO_TIMEOUT_US);
SonarReader sonarLeft(SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_ECHO_TIMEOUT_US);
SonarReader sonarRight(SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN, SONAR_ECHO_TIMEOUT_US);
SonarReader sonarRear(SONAR_REAR_TRIG_PIN, SONAR_REAR_ECHO_PIN, SONAR_ECHO_TIMEOUT_US);
// Heading heading;
const uint8_t system_id = 1;
const uint8_t component_id = 200;



// Global I2C task handle (optional)
TaskHandle_t i2cTaskHandle = nullptr;

void i2cTask(void *pvParameters) {
    // Create display and IMU objects locally – they own the I2C bus
    DisplayController display;
    if (!display.begin()) {
        DEBUG_SERIAL.println("Display init failed");
    }

    Heading heading;
    if (!heading.begin()) {
        DEBUG_SERIAL.println("IMU init failed");
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
    Serial.begin(115200);        // native USB CDC, binary MAVLink only
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD); // UART0, human readable logs
    steer.begin(SERVO_FREQUENCY_HZ);
    steer.to(STEERING_CENTER_ANGLE);
    motor.begin(MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    encoder.begin(ENCODER_SAMPLE_INTERVAL_MS);
    Wire.begin(I2C_SDA, I2C_SCL);
    
    sharedMutex = xSemaphoreCreateMutex();

    sonarFront.begin();
    sonarLeft.begin();
    sonarRight.begin();
    sonarRear.begin();

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


    // --- Sonars: one sensor per loop() call so a pulseIn timeout never
    // blocks the loop for more than ~25 ms at a time ---
    static uint8_t sonarCm[4] = {255, 255, 255, 255};
    static uint8_t sonarIndex = 0;
    switch (sonarIndex) {
        case 0: sonarCm[0] = sonarFront.readCm(); break; // sonar_1
        case 1: sonarCm[1] = sonarLeft.readCm();  break; // sonar_2
        case 2: sonarCm[2] = sonarRight.readCm(); break; // sonar_3
        case 3: sonarCm[3] = sonarRear.readCm();  break; // sonar_4
    }
    sonarIndex = (sonarIndex + 1) % 4;

    // --- Send encoder/steering/sonar telemetry to ROS2 ---
    static unsigned long lastSensorTxMs = 0;
    static long lastSentEncoderCount = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastSensorTxMs >= SENSOR_TX_INTERVAL_MS) {
        lastSensorTxMs = nowMs;

        long currentCount = encoder.count();
        long delta = currentCount - lastSentEncoderCount; // ticks since last send
        lastSentEncoderCount = currentCount;
        long deltaMagnitude = delta < 0 ? -delta : delta;

        float rpm = encoder.rpm();
        float rpmMagnitude = rpm < 0 ? -rpm : rpm;

        uint8_t encoderCountField = (uint8_t)constrain(deltaMagnitude, 0L, 255L);
        uint8_t encoderSpeedField = (uint8_t)constrain((long)rpmMagnitude, 0L, 255L);
        uint8_t encoderDirectionField = 0; // 0 = stopped
        if (encoder.direction() > 0) encoderDirectionField = 1;      // forward
        else if (encoder.direction() < 0) encoderDirectionField = 2; // reverse

        mavlink_message_t txMsg;
        uint8_t txBuf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack(system_id, component_id, &txMsg,
            encoderCountField, encoderSpeedField, encoderDirectionField, steer.getAngle(),
            sonarCm[0], sonarCm[1], sonarCm[2], sonarCm[3]);
        uint16_t txLen = mavlink_msg_to_send_buffer(txBuf, &txMsg);
        Serial.write(txBuf, txLen);
    }

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



