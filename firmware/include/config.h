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

// I2C device bring up and recovery.
// Nothing on the bus is initialised once and then trusted forever. On a cold
// power up the esp32 reaches the I2C task a few hundred ms in, while the BNO055
// needs 650 ms of power on reset before it answers at all - and Adafruit's
// begin() gives up on the very first NACK. A single failed attempt used to mean
// a dark screen and a dead IMU until the next reboot, even though both devices
// were perfectly healthy a moment later (which is why an I2C scanner, scanning
// in a loop, always finds them). The same applies mid run: a brownout when the
// motor kicks in can knock a device off the bus. So a device that is not up gets
// retried, and a device that stops answering is re-initialised.
// Adafruit_BNO055::begin() already spends up to 850 ms retrying detection on its
// own, which covers the 650 ms power on reset by itself - this interval is what
// governs a longer absence (unplugged sensor, brownout). Retrying much faster
// than this just parks the task inside a failing begin() and starves the screen.
const unsigned long I2C_INIT_RETRY_INTERVAL_MS = 2000;
const unsigned long I2C_HEALTH_CHECK_INTERVAL_MS = 1000;

// a full 128x64 frame is ~25 ms of bus time, so 50 ms already spends half the
// bus on the screen. lower this before raising it.
const unsigned long DISPLAY_REFRESH_INTERVAL_MS = 50;
const unsigned long IMU_SAMPLE_INTERVAL_MS = 10;

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
