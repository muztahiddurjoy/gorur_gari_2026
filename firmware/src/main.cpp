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
#include "button_handler.h"
#include "status_led.h"

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
SonarReader sonarFront(SONAR_FRONT_TRIG_PIN, SONAR_FRONT_ECHO_PIN, SONAR_ECHO_TIMEOUT_US, SONAR_FRONT_ENABLED);
SonarReader sonarLeft(SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_ECHO_TIMEOUT_US, SONAR_LEFT_ENABLED);
SonarReader sonarRight(SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN, SONAR_ECHO_TIMEOUT_US, SONAR_RIGHT_ENABLED);
SonarReader sonarRear(SONAR_REAR_TRIG_PIN, SONAR_REAR_ECHO_PIN, SONAR_ECHO_TIMEOUT_US, SONAR_REAR_ENABLED);
ButtonHandler button(BUTTON_PIN);
StatusLed status_led(STATUS_LED_PIN);
// Heading heading;
const uint8_t system_id = 1;
const uint8_t component_id = 200;



// Global I2C task handle (optional)
TaskHandle_t i2cTaskHandle = nullptr;

void i2cTask(void *pvParameters) {
    // Create display and IMU objects locally – they own the I2C bus
    DisplayController display;
    Heading heading;

    TickType_t lastDisplayUpdate = xTaskGetTickCount();
    const TickType_t displayInterval = pdMS_TO_TICKS(DISPLAY_REFRESH_INTERVAL_MS);
    const TickType_t imuInterval = pdMS_TO_TICKS(IMU_SAMPLE_INTERVAL_MS);
    const TickType_t initRetryInterval = pdMS_TO_TICKS(I2C_INIT_RETRY_INTERVAL_MS);
    const TickType_t healthCheckInterval = pdMS_TO_TICKS(I2C_HEALTH_CHECK_INTERVAL_MS);

    TickType_t lastIMUUpdate = xTaskGetTickCount();
    TickType_t lastHealthCheck = xTaskGetTickCount();
    // one interval back, so both devices get their first attempt straight away
    TickType_t lastDisplayInit = xTaskGetTickCount() - initRetryInterval;
    TickType_t lastIMUInit = lastDisplayInit;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // --- Bring up whatever is not up yet. On a cold boot the panel and the
        // BNO055 may still be in their own power on reset, so a failed attempt
        // means "not yet", never "give up" ---
        if (!display.isReady() && now - lastDisplayInit >= initRetryInterval) {
            lastDisplayInit = now;
            if (display.begin()) {
                DEBUG_SERIAL.println("Display ready");
            }
        }
        if (!heading.isReady() && now - lastIMUInit >= initRetryInterval) {
            lastIMUInit = now;
            heading.begin();  // logs its own outcome
        }

        // --- Watch for a device dropping off the bus mid run (brownout when the
        // motor kicks in, a loose jumper). Marking it lost hands it back to the
        // retry path above, so it recovers without a reboot ---
        if (now - lastHealthCheck >= healthCheckInterval) {
            lastHealthCheck = now;
            if (display.isReady() && !display.isResponding()) {
                DEBUG_SERIAL.println("Display stopped responding, re-initialising");
                display.markLost();
            }
            if (heading.isReady() && !heading.isResponding()) {
                DEBUG_SERIAL.println("BNO055 stopped responding, re-initialising");
                heading.markLost();
            }
        }

        // --- Update IMU (every IMU_SAMPLE_INTERVAL_MS) ---
        if (heading.isReady() && now - lastIMUUpdate >= imuInterval) {
            lastIMUUpdate = now;
            heading.update();
            float yaw = heading.getHeading();

            // Write heading to shared struct
            if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
                shared.heading = yaw;
                xSemaphoreGive(sharedMutex);
            }
        }

        // --- Update Display (every DISPLAY_REFRESH_INTERVAL_MS) ---
        if (display.isReady() && now - lastDisplayUpdate >= displayInterval) {
            lastDisplayUpdate = now;

            // Read display strings from shared struct
            String speed, encoder, steering, velData;
            float yaw = 0.0f;
            if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
                speed = shared.speedText;
                encoder = shared.encoderText;
                steering = shared.steeringText;
                velData = shared.vel_data;
                // taken under the same lock as the strings, so the screen never
                // shows a half written float
                yaw = shared.heading;
                xSemaphoreGive(sharedMutex);
            }

            // Draw everything
            display.clear();
            display.displayText(speed, 0, 0, 1);
            display.displayText(encoder, 0, 10, 1);
            display.displayText(steering, 0, 20, 1);
            display.displayText("Yaw: " + String(yaw), 0, 30, 1);
            display.displayText(velData, 0, 40, 1);
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
    button.begin();
    // stays off until the ROS2 bridge announces itself over serial, see the
    // connect message handling in loop()
    status_led.begin();
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

     float headingDeg = 0.0f;
     if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
        shared.speedText = "Speed: " + String(motor.speed());
        shared.encoderText = "Encoder: " + String(encoder.count());
        shared.steeringText = "Steering: " + String(steer.getAngle());
        // written by the I2C task on the other core, so only read it in here
        headingDeg = shared.heading;
        xSemaphoreGive(sharedMutex);
    }


    // --- Sonars: one sensor per loop() call so a pulseIn timeout never blocks
    // the loop for more than SONAR_ECHO_TIMEOUT_US at a time. readCm() returns
    // straight away for a sonar disabled in config.h, so unplugged sensors cost
    // nothing and simply keep reporting SONAR_NO_READING_CM ---
    static uint8_t sonarCm[4] = {SONAR_NO_READING_CM, SONAR_NO_READING_CM,
                                 SONAR_NO_READING_CM, SONAR_NO_READING_CM};
    static uint8_t sonarIndex = 0;
    switch (sonarIndex) {
        case 0: sonarCm[0] = sonarFront.readCm(); break; // sonar_1
        case 1: sonarCm[1] = sonarLeft.readCm();  break; // sonar_2
        case 2: sonarCm[2] = sonarRight.readCm(); break; // sonar_3
        case 3: sonarCm[3] = sonarRear.readCm();  break; // sonar_4
    }
    sonarIndex = (sonarIndex + 1) % 4;

    // --- Button: poll every loop, but telemetry only goes out every
    // SENSOR_TX_INTERVAL_MS. Latch the press edge so a tap shorter than that
    // interval still reaches ROS2 instead of falling between two frames ---
    static bool buttonPressLatch = false;
    if (button.isPressed()) {
        buttonPressLatch = true;
        DEBUG_SERIAL.println("Button pressed!");
        // count the start delay out on the LED. ROS2 starts its own timer off
        // the same press, so both ends run the same countdown - the blinks are
        // over at the moment the car begins to move.
        status_led.blink(START_BLINK_COUNT, START_BLINK_INTERVAL_MS);
    }
    // advances the countdown blinks, no-op the rest of the time
    status_led.update();

    // --- Send encoder/steering/heading/sonar/button telemetry to ROS2 ---
    static unsigned long lastSensorTxMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastSensorTxMs >= SENSOR_TX_INTERVAL_MS) {
        lastSensorTxMs = nowMs;

        // raw cumulative tick count, the same value the OLED prints
        int32_t encoderCountField = (int32_t)encoder.count();

        float rpm = encoder.rpm();
        float rpmMagnitude = rpm < 0 ? -rpm : rpm;

        uint8_t encoderSpeedField = (uint8_t)constrain((long)rpmMagnitude, 0L, 255L);
        uint8_t encoderDirectionField = 0; // 0 = stopped
        if (encoder.direction() > 0) encoderDirectionField = 1;      // forward
        else if (encoder.direction() < 0) encoderDirectionField = 2; // reverse

        // raw heading, the same value the OLED prints (unwrapped degrees)
        float headingField = headingDeg;

        // held down right now, or tapped since the last frame went out
        uint8_t buttonField = (button.isDown() || buttonPressLatch) ? 1 : 0;
        buttonPressLatch = false;

        mavlink_message_t txMsg;
        uint8_t txBuf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack(system_id, component_id, &txMsg,
            encoderCountField, encoderSpeedField, encoderDirectionField, steer.getAngle(),
            headingField, sonarCm[0], sonarCm[1], sonarCm[2], sonarCm[3], buttonField);
        uint16_t txLen = mavlink_msg_to_send_buffer(txBuf, &txMsg);
        Serial.write(txBuf, txLen);
    }


    while(Serial.available()>0){
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
            // mcu_bridge sends this once, right after it opens the port. it is the
            // only announcement we get that a ROS2 side actually exists, so it is
            // what drives the status LED. if a start countdown happens to be
            // blinking, set() just records the level and the LED lands on it once
            // the countdown finishes.
            else if(received_msg.msgid == MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg){
                uint8_t connected = mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_get_connected(&received_msg);
                status_led.set(connected != 0);
                DEBUG_SERIAL.println(connected ? "ROS2 bridge connected" : "ROS2 bridge disconnected");
            }
        }
    }


}



