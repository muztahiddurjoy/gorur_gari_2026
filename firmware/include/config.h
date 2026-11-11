#ifndef CONFIG_H
#define CONFIG_H

// The firmware applies whatever throttle and steering arrive on the wire -
// the drive limits (steering swing/clamps, throttle scaling) are enforced on
// the ROS2 side only, see the mcu_bridge section of ros2_ws/config/bot_config.yaml.

//PWM timers
// The ESP32-S3 LEDC peripheral has 4 timers; setup() hands all of them to
// ESP32PWM (allocateTimer(0..3)) before the motor or the servo attaches.
const int LEDC_TIMER_COUNT = 4;

//servo (MG996R)
const int SERVO_FREQUENCY_HZ = 50;
// The wire angle that means "straight" (mcu_bridge's steering_center_deg).
// Boot position, and where the steering goes when commands stop.
const int STEERING_CENTER_ANGLE = 90;
// Steering limits as servo pulse widths, in microseconds. Wire angle 0 maps
// to STEERING_MIN_US, 90 to STEERING_CENTER_US and 180 to STEERING_MAX_US
// (piecewise linear, see lib/steering_control), and the servo is never sent a
// pulse outside MIN..MAX whatever arrives on the wire.
//
// These are deliberately conservative placeholders (+/-400 us around a generic
// 1500 us centre, roughly +/-36 deg of MG996R horn travel). CALIBRATE ON THE
// BENCH before driving: find the pulse where the wheels point straight
// (CENTER), then walk each side out until the linkage is about to touch its
// stop and back off a little (MIN for wire angle 0 / full left, MAX for 180 /
// full right). If the wheels turn the wrong way, swap MIN and MAX.
const int STEERING_MIN_US = 1100;
const int STEERING_CENTER_US = 1500;
const int STEERING_MAX_US = 1900;

//motor (BTS7960, see lib/motor_control)
const uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000; // above hearing range so the motor stays quiet; BTS7960 max is 25 kHz
const uint8_t MOTOR_PWM_RESOLUTION_BITS = 8;   // duty is 0..255, 255 = fully on
// The throttle field on the wire is an int8 (mcu_bridge clamps it to
// -128..127). +/-MOTOR_COMMAND_MAX is full duty in that direction, so the whole
// command range now reaches the whole duty range (-128 counts as -127). The
// old TB6612 code wrote the raw command into an 8-bit duty, so a full 127
// only ever reached ~50 %: the same /cmd_vel now drives about twice as hard.
const int MOTOR_COMMAND_MAX = 127;
// How fast the motor output may change, in % of full duty per second. 500 %/s
// is 0 -> full in 200 ms, and a full forward -> full reverse swing takes 400 ms,
// braking through zero on the way. The safety stop (CMD_TIMEOUT_MS) ignores
// this and brakes at once. <= 0 turns the ramp off.
const float MOTOR_SLEW_PERCENT_PER_S = 500.0f;

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
// No sonars are fitted on this hardware revision, but the mcu_to_ros2 MAVLink
// message still has its four sonar_* fields, so they always carry the "no
// reading" value. mcu_bridge already treats 255 as "nothing measured"
// (SONAR_NO_READING_CM there) and publishes nothing for sonars it has disabled.
const uint8_t SONAR_WIRE_NO_READING_CM = 255;

//start button (the BOOT button on GPIO0, see pins.h)
const unsigned long START_BUTTON_DEBOUNCE_MS = 30;
// Pressing the button arms the run. The ROS2 side (autonomy/disparity_extender)
// holds the car still for START_BLINK_COUNT * START_BLINK_INTERVAL_MS after the
// press, and the status LED blinks that delay out so the countdown is visible on
// the car. Keep these in step with start_delay_sec in
// ros2_ws/config/disparity_extender_params.yaml - 3 * 1000 ms = 3 s.
const uint8_t START_BLINK_COUNT = 3;
const unsigned long START_BLINK_INTERVAL_MS = 1000;

//onboard WS2812 status pixel
// The second status light, next to the plain LED above: red until the ROS2
// bridge announces itself, green once it has, flashing blue while the motor
// drives and flashing purple while it drives with the wheels turned.
// 64/255 is bright enough to read across the pit table without being the
// blinding white-ish blob the raw pixel is at close range.
const uint8_t RGB_STATUS_LED_BRIGHTNESS = 64;
// one full on+off flash. 300 ms reads clearly as "flashing" without strobing,
// and is short enough that a two second drive still shows several cycles.
const unsigned long RGB_STATUS_FLASH_PERIOD_MS = 300;
// how far off STEERING_CENTER_ANGLE still counts as driving straight. mcu_bridge
// rounds its steering command to whole degrees, so without a little slack a
// nearly straight run would flicker between blue and purple. set to 0 to make
// it literal - anything but dead centre is a turn.
const uint8_t RGB_STEERING_CENTER_TOLERANCE_DEG = 2;

//ROS2 telemetry
const unsigned long SENSOR_TX_INTERVAL_MS = 50; // how often the mcu_to_ros2 sensor message is sent

//UART link to the Pi (Serial1, 8N1). Must match MCU_BAUDRATE in mcu_bridge.py.
const unsigned long PI_UART_BAUD = 115200;
// Both sizes must be above the 128 byte hardware FIFO. The TX ring buffer is
// what makes Serial1.write() return at once instead of waiting for the wire
// (one telemetry frame is 28 bytes, a STATUSTEXT up to 66). The RX side holds
// ~0.5 s of the Pi's traffic, far more than loop() ever leaves unread.
const size_t PI_UART_TX_BUFFER_BYTES = 512;
const size_t PI_UART_RX_BUFFER_BYTES = 512;

#endif
