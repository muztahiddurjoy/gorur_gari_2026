#ifndef CONFIG_H
#define CONFIG_H

//servo
const int SERVO_FREQUENCY_HZ = 50;
const int STEERING_CENTER_ANGLE = 90;
const int STEERING_MAX_ANGLE = 35; // how far the wheels swing either side of centre

//motor
const int MOTOR_PWM_FREQUENCY_HZ = 20000; // above hearing range so the motor stays quiet
const int MOTOR_PWM_RESOLUTION_BITS = 8;  // duty is 0..255
const int MOTOR_MAX_SPEED = 255;

//encoder
const int ENCODER_COUNTS_PER_REV = 1320;   // 11 ppr * 4 edges * 30:1 gearbox, retune for your motor
const int ENCODER_SAMPLE_INTERVAL_MS = 50;

//I2C addresses
const uint8_t OLED_ADDRESS = 0x3C; // I2C address for the OLED display
const uint8_t BNO055_ADDRESS = 0x29; // I2C address for the BNO055 IMU

//sonar
// flip one of these to true once that sonar is physically wired up. a disabled
// sonar never has its pins configured and is never pulsed, so it costs nothing
// and always reports SONAR_NO_READING_CM. keep these in step with the
// sonar_*_enabled parameters in ros2_ws/contols/contols/mcu_bridge.py.
const bool SONAR_FRONT_ENABLED = false;
const bool SONAR_LEFT_ENABLED = false;
const bool SONAR_RIGHT_ENABLED = false;
const bool SONAR_REAR_ENABLED = false;

// 255 cm is the furthest the uint8_t "cm" wire field can carry, and that round
// trip is 255*58 = 14790 us. Waiting longer than that only buys readings we
// would clamp to 255 anyway, while every timed out sonar stalls loop() for the
// full duration and delays the next throttle/steering command.
const unsigned long SONAR_ECHO_TIMEOUT_US = 15000;

//ROS2 telemetry
const unsigned long SENSOR_TX_INTERVAL_MS = 50; // how often the mcu_to_ros2 sensor message is sent

#endif
