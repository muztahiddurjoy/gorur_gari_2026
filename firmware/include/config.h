#ifndef CONFIG_H
#define CONFIG_H

// The firmware applies whatever throttle and steering arrive on the wire -
// the drive limits (steering swing/clamps, throttle scaling) are enforced on
// the ROS2 side, see the mcu_bridge section of ros2_ws/config/bot_config.yaml.
// The two exceptions are hardware protection: the steering pulse limits
// (STEERING_*_US) and the drive command timeout below.

//drive command safety
// The car moves only while valid drive commands keep arriving from the Pi.
// After boot the motor is braked until the first valid
// gorur_gari_ros2_to_mcu_msg, and whenever the newest one is older than this
// the motor brakes at once and the steering centres, until the next one.
// Heartbeats, connect/timer frames and anything with a bad CRC never count
// (lib/drive_link). mcu_bridge sends drive commands at 50 Hz (every 20 ms)
// while /cmd_vel is fresh, so 100 ms rides through four lost frames in a row.
// Do not lower the bridge's rate without raising this.
const uint32_t CMD_TIMEOUT_MS = 100;
// the timeout / resume messages on the USB debug port print at most this often
const uint32_t DRIVE_LOG_MIN_INTERVAL_MS = 1000;

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

//I2C bus (SDA GPIO8 / SCL GPIO9), shared by the BOTGyro and the OLED
// 400 kHz keeps a full OLED frame to ~25 ms of bus time, during which the gyro
// cannot be read. Both devices are clocked from this one value (the OLED
// driver re-sets the bus clock around every frame, so it is given the same
// number). If gyro reads turn out unreliable at this speed, the firmware drops
// the whole bus to I2C_FALLBACK_CLOCK_HZ by itself (see below).
const uint32_t I2C_CLOCK_HZ = 400000;
// The fallback: after I2C_FALLBACK_AFTER_FAILURES gyro faults (or begin()
// failures while something still ACKs the gyro's address - a sensor that is
// simply not plugged in does not count), the bus is re-clocked to 100 kHz for
// the rest of the run, via BOTGyro::setI2CClock() for its re-inits,
// Wire.setClock() for the bus and a rebuilt OLED driver. Logged on USB and
// sent to the Pi as a STATUSTEXT. It never switches back up by itself.
const uint32_t I2C_FALLBACK_CLOCK_HZ = 100000;
const uint8_t I2C_FALLBACK_AFTER_FAILURES = 3;

//I2C addresses
const uint8_t OLED_ADDRESS = 0x3C;  // SSD1306
const uint8_t IMU_ADDRESS = 0x6A;   // BOTGyro with SA0 low; 0x6B (BOTGYRO_ADDR_HIGH) if SA0 is tied high

//IMU (BOTGyro: ASM330LHH gyro + accelerometer, no magnetometer, lib/heading)
// 416 Hz: fine time steps for the yaw integration, and calibration is
// IMU_CALIBRATION_SAMPLES / 416 = ~1.5 s of standing still.
const float IMU_ODR_HZ = 416.0f;
// a small car spins well under 1000 deg/s even in a crash; 2000 would halve
// the resolution for nothing, 500 could clip in one
const int32_t IMU_GYRO_FULL_SCALE_DPS = 1000;
const int32_t IMU_ACCEL_FULL_SCALE_G = 4;     // bumps and kerbs stay unclipped for pitch/roll
const uint16_t IMU_CALIBRATION_SAMPLES = 600;
const float IMU_ATTITUDE_ALPHA = 0.98f;       // pitch/roll complementary filter, weight on the gyro
// The ROS nodes expect the heading to grow CLOCKWISE seen from above (the
// BNO055 compass convention; heading_clockwise: true in bot_config.yaml). A
// right-handed gyro mounted with its Z axis UP reads counter-clockwise as
// positive, hence -1. If the board is mounted upside down, use +1. Check it on
// the bench: turn the car clockwise by hand and the OLED yaw must go UP.
const float IMU_YAW_SIGN = -1.0f;
// Samples further apart than this (a stalled bus, a fault) are not
// integrated across: the rate in between is unknown. An OLED frame stalls the
// gyro for ~25 ms at 400 kHz, ~100 ms at 100 kHz, so this sits just above.
const float IMU_MAX_INTEGRATION_GAP_S = 0.12f;
// begin() retry while the sensor is missing. BOTGyro re-initialises a sensor
// that faults mid-run by itself, once a second.
const uint32_t IMU_BEGIN_RETRY_MS = 3000;
// Drive commands are ignored (the motor stays braked, the watchdog is not
// fed) until the gyro's first calibration has finished, so the car stands
// still while it measures its zero-rate bias - but for no longer than this
// after boot, so a missing or broken IMU cannot keep the car from driving.
// The gate only ever closes at boot; it never closes again once open.
const uint32_t IMU_CAL_GATE_MAX_MS = 5000;
// While the IMU is not usable (not found, faulted, uncalibrated) the MAVLink
// STATUSTEXT reporting it is repeated this often; every change is sent at once.
const uint32_t IMU_STATUSTEXT_REPEAT_MS = 5000;

// Device bring up and recovery on the bus.
// Nothing on the bus is initialised once and then trusted forever: on a cold
// power up a device may still be in its own power on reset when the I2C task
// first tries it, and a brownout when the motor kicks in can knock one off the
// bus mid run. So a device that is not up gets retried, and one that stops
// answering is re-initialised. These two govern the OLED; the IMU retries on
// IMU_BEGIN_RETRY_MS and its library's own fault recovery.
const unsigned long I2C_INIT_RETRY_INTERVAL_MS = 2000;
const unsigned long I2C_HEALTH_CHECK_INTERVAL_MS = 1000;

// A full 128x64 frame is ~25 ms of bus time at 400 kHz, and the gyro cannot be
// read while it goes out, so the yaw integration bridges each frame on two
// samples. 200 ms (5 Hz) keeps that to ~12 % of the time; the screen is for
// people, it does not need to be faster.
const unsigned long DISPLAY_REFRESH_INTERVAL_MS = 200;
// at I2C_FALLBACK_CLOCK_HZ a frame takes ~100 ms of bus time, so slow down
const unsigned long DISPLAY_REFRESH_INTERVAL_FALLBACK_MS = 1000;

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

//status LED patterns (priority: start countdown > IMU pattern > link level)
// IMU calibrating, keep the car still: fast even blink, 5 Hz
const unsigned long STATUS_LED_CAL_PERIOD_MS = 200;
// IMU not found / faulted / uncalibrated: one short blip a second
const unsigned long STATUS_LED_FAULT_PERIOD_MS = 1000;
const unsigned long STATUS_LED_FAULT_ON_MS = 100;

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

//USB debug output
// One status line (drive state, motor, steering, IMU, yaw/pitch/roll, and the
// longest gap between two loop() passes since the last line) per interval.
// 0 turns it off. Timeout/resume and IMU events are logged regardless.
const uint32_t DEBUG_STATUS_INTERVAL_MS = 1000;

//MAVLink link to the Pi: native USB CDC on the devkit's "USB" port
// (the S3's USB-Serial/JTAG controller; ARDUINO_USB_MODE=1 and
// ARDUINO_USB_CDC_ON_BOOT=1 in platformio.ini make it `Serial`). Carries
// MAVLink ONLY - see debug_serial.h. Helpers in include/mav_serial.h.
#define MAV_SERIAL Serial
// USB CDC ignores the baud rate; kept equal to MCU_BAUDRATE in mcu_bridge.py
const unsigned long MAV_SERIAL_BAUD = 115200;
// Ring buffers (the core defaults to 256 each). TX: a telemetry frame is 28
// bytes every 50 ms and a STATUSTEXT up to 66, so 1 KB never fills while the
// Pi reads; when it does fill (port closed, Pi busy) whole frames are dropped,
// never half frames and never by waiting. RX: about a second of the Pi's
// traffic, far more than loop() ever leaves unread.
const size_t MAV_SERIAL_TX_BUFFER_BYTES = 1024;
const size_t MAV_SERIAL_RX_BUFFER_BYTES = 1024;
// 0 = the core's USB write() never waits for the host (its default is 100 ms,
// the whole drive-command timeout)
const uint32_t MAV_SERIAL_TX_TIMEOUT_MS = 0;

#endif
