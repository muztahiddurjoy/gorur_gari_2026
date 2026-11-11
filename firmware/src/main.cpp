#include <Arduino.h>
#include "steering_control.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "Wire.h"
#include "display_control.h"
#include "heading.h"
#include "pins.h"
#include "config.h"
#include "debug_serial.h"
#include "shared_data.h"
#include "button_handler.h"
#include "status_led.h"
#include "rgb_status_led.h"
#include "run_timer.h"
#include "drive_link.h"
#include "rate_limited_log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// both MAVLink dialects (and STATUSTEXT), see the include order notes there
#include "mavlink_link.h"



SteeringController steer(STEERING_SERVO_PIN);
MotorController motor(MOTOR_RPWM_PIN, MOTOR_LPWM_PIN, MOTOR_EN_PIN);
EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTS_PER_REV);
ButtonHandler button(START_BUTTON_PIN, START_BUTTON_DEBOUNCE_MS);
StatusLed status_led(STATUS_LED_PIN);
RgbStatusLed rgb_led(RGB_STATUS_LED_PIN);
const uint8_t system_id = 1;
const uint8_t component_id = 200;

// the MAVLink link to the Raspberry Pi: UART1 on GPIO40/41 (pins.h). The USB
// port (Serial) only ever carries debug text, see debug_serial.h.
#define PI_SERIAL Serial1

// A MAVLink STATUSTEXT to the Pi, which mcu_bridge logs as "MCU: <text>" at
// the matching level. This is how the firmware reports events now that no
// text may go out on the link (debug_serial.h). Called from loop() and
// setup() only, so the link keeps a single writer. At most 50 characters.
static void statusTextf(uint8_t severity, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void statusTextf(uint8_t severity, const char *fmt, ...) {
    char text[MAVLINK_STATUSTEXT_TEXT_LEN + 1];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    DBG_PRINTF("[mcu] %s\n", text);

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_pack_statustext(system_id, component_id, &msg, severity, text);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    PI_SERIAL.write(buf, len);
}



// --- drive command safety (see CMD_TIMEOUT_MS in config.h) -------------------
// driveActive is true only while a fresh valid drive command is in force. It
// starts false, so the motor is braked from boot until the first valid one.
DriveWatchdog driveWatchdog(CMD_TIMEOUT_MS);
bool driveActive = false;
RateLimitedLog driveTimeoutLog(DRIVE_LOG_MIN_INTERVAL_MS);
RateLimitedLog driveResumeLog(DRIVE_LOG_MIN_INTERVAL_MS);

// A valid drive command arrived: refresh the watchdog and apply it.
static void applyDriveCommand(const DriveCommand &cmd, uint32_t nowMs) {
    bool first = !driveWatchdog.everFed();
    uint32_t gapMs = driveWatchdog.ageMs(nowMs);
    driveWatchdog.feed(nowMs);

    // full command range -> full duty range, see MOTOR_COMMAND_MAX
    motor.setTarget((float)constrain((int)cmd.throttle, -MOTOR_COMMAND_MAX, MOTOR_COMMAND_MAX)
                    / (float)MOTOR_COMMAND_MAX);
    steer.to(cmd.steering);

    if (!driveActive) {
        driveActive = true;
        if (driveResumeLog.shouldLog(nowMs)) {
            if (first) {
                statusTextf(STATUSTEXT_SEVERITY_INFO, "Drive: first valid command, motor released");
            } else {
                statusTextf(STATUSTEXT_SEVERITY_INFO, "Drive: resumed after %lu ms (+%lu more)",
                            (unsigned long)gapMs, (unsigned long)driveResumeLog.suppressed());
            }
        }
    }
}

// Runs every loop() pass, after the serial input has been read. No valid
// drive command for CMD_TIMEOUT_MS -> brake at once (no ramp) and centre the
// steering; stay that way until the next valid command.
static void enforceDriveTimeout(uint32_t nowMs) {
    if (driveActive && !driveWatchdog.alive(nowMs)) {
        driveActive = false;
        steer.center();
        if (driveTimeoutLog.shouldLog(nowMs)) {
            statusTextf(STATUSTEXT_SEVERITY_WARNING, "Drive: no command for %lu ms, braked (+%lu more)",
                        (unsigned long)driveWatchdog.ageMs(nowMs),
                        (unsigned long)driveTimeoutLog.suppressed());
        }
    }
    if (!driveActive) {
        // immediate and idempotent: the bridge pins are only rewritten if
        // something had changed them
        motor.brake();
    }
}


// --- the I2C side: IMU + OLED, on core 0 -------------------------------------

static HeadingConfig imuConfig() {
    HeadingConfig c;
    c.i2cAddress = IMU_ADDRESS;
    c.i2cClockHz = I2C_CLOCK_HZ;
    c.odrHz = IMU_ODR_HZ;
    c.gyroFullScaleDps = IMU_GYRO_FULL_SCALE_DPS;
    c.accelFullScaleG = IMU_ACCEL_FULL_SCALE_G;
    c.calibrationSamples = IMU_CALIBRATION_SAMPLES;
    c.attitudeAlpha = IMU_ATTITUDE_ALPHA;
    c.yawSign = IMU_YAW_SIGN;
    c.maxIntegrationGapS = IMU_MAX_INTEGRATION_GAP_S;
    c.beginRetryMs = IMU_BEGIN_RETRY_MS;
    return c;
}

static const char *driveStateLine(uint8_t state) {
    switch (state) {
        case DRIVE_ACTIVE:  return "Cmd: OK";
        case DRIVE_STOPPED: return "Cmd: STOP (no cmd)";
        case DRIVE_IMU_HOLD: return "Cmd: held, IMU cal";
        default:            return "Cmd: waiting";
    }
}

// The two IMU lines of the OLED
static void imuLines(const SharedData &d, char *line1, char *line2, size_t len) {
    switch ((ImuStatus)d.imuStatus) {
        case ImuStatus::Calibrating:
            snprintf(line1, len, "IMU calibrating,");
            snprintf(line2, len, "keep still!");
            break;
        case ImuStatus::Ready:
            snprintf(line1, len, "IMU OK %u Hz", (unsigned)d.imuSampleRateHz);
            snprintf(line2, len, "I2C %lu kHz", (unsigned long)(d.i2cClockHz / 1000));
            break;
        case ImuStatus::Uncalibrated:
            snprintf(line1, len, "IMU NOT CALIBRATED");
            snprintf(line2, len, "yaw held");
            break;
        case ImuStatus::Fault:
            snprintf(line1, len, "IMU FAULT");
            snprintf(line2, len, "retrying...");
            break;
        case ImuStatus::NotFound:
        default:
            snprintf(line1, len, "IMU not found 0x%02X", IMU_ADDRESS);
            snprintf(line2, len, "retrying...");
            break;
    }
}

// Global I2C task handle (optional)
TaskHandle_t i2cTaskHandle = nullptr;

// Owns the I2C bus: nothing on core 1 touches Wire. The IMU is serviced on
// EVERY pass (gyro.update() inside Heading::service), the OLED is redrawn
// every DISPLAY_REFRESH_INTERVAL_MS. Anything slow on the bus - an OLED frame,
// BOTGyro's 120 ms begin()/re-init - only ever stalls this task.
void i2cTask(void *pvParameters) {
    DisplayController display;
    Heading heading(Wire, imuConfig());

    TickType_t lastDisplayUpdate = xTaskGetTickCount();
    TickType_t displayInterval = pdMS_TO_TICKS(DISPLAY_REFRESH_INTERVAL_MS);
    const TickType_t initRetryInterval = pdMS_TO_TICKS(I2C_INIT_RETRY_INTERVAL_MS);
    const TickType_t healthCheckInterval = pdMS_TO_TICKS(I2C_HEALTH_CHECK_INTERVAL_MS);

    TickType_t lastHealthCheck = xTaskGetTickCount();
    // one interval back, so the display gets its first attempt straight away
    TickType_t lastDisplayInit = xTaskGetTickCount() - initRetryInterval;

    // from loop(): may the gyro (re)calibrate, i.e. is the car meant to be still
    bool calibrationAllowed = true;
    uint32_t busClockHz = I2C_CLOCK_HZ;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // --- IMU, every pass: begin() retries, BOTGyro's state machine
        // (sampling, non-blocking calibration, fault recovery), yaw ---
        heading.service(calibrationAllowed);

        // --- I2C fallback: gyro reads unreliable at the full clock -> re-clock
        // the whole bus once, for the rest of the run (see config.h) ---
        if (busClockHz != I2C_FALLBACK_CLOCK_HZ) {
            uint32_t unreliable = heading.faultCount() + heading.failedBeginsWhilePresent();
            if (unreliable >= I2C_FALLBACK_AFTER_FAILURES) {
                // reported to the Pi by loop() (reportImuStatus) as a STATUSTEXT
                DBG_PRINTF("[i2c] %lu gyro faults/failed inits at %lu Hz, dropping the bus to %lu Hz\n",
                                    (unsigned long)unreliable, (unsigned long)busClockHz,
                                    (unsigned long)I2C_FALLBACK_CLOCK_HZ);
                busClockHz = I2C_FALLBACK_CLOCK_HZ;
                heading.setI2CClock(busClockHz);   // BOTGyro's next begin()
                Wire.setClock(busClockHz);         // the bus, now (BOTGyro's own re-init uses it)
                display.markLost();                // rebuilt at the new clock by begin()
                displayInterval = pdMS_TO_TICKS(DISPLAY_REFRESH_INTERVAL_FALLBACK_MS);
            }
        }

        // --- swap results with loop(). Waiting for the lock is fine on this
        // core; loop() never holds it for more than a few copies ---
        if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
            shared.heading = heading.yawDeg();
            shared.pitchDeg = heading.pitchDeg();
            shared.rollDeg = heading.rollDeg();
            shared.yawRateDps = heading.yawRateDps();
            shared.accelXg = heading.accelXg();
            shared.accelYg = heading.accelYg();
            shared.accelZg = heading.accelZg();
            shared.imuStatus = (uint8_t)heading.status();
            shared.imuBeginTried = heading.beginAttempted();
            shared.imuEverCalibrated = heading.everCalibrated();
            shared.imuSampleRateHz = heading.sampleRateHz();
            shared.i2cClockHz = busClockHz;
            // the car is meant to be still unless a drive command is in force
            calibrationAllowed = (shared.driveState != DRIVE_ACTIVE);
            xSemaphoreGive(sharedMutex);
        }

        // --- Bring the OLED up if it is not up yet. On a cold boot the panel
        // may still be in its own power on reset, so a failed attempt means
        // "not yet", never "give up" ---
        if (!display.isReady() && now - lastDisplayInit >= initRetryInterval) {
            lastDisplayInit = now;
            if (display.begin(busClockHz)) {
                DBG_PRINTLN("Display ready");
            }
        }

        // --- Watch for the panel dropping off the bus mid run (brownout when
        // the motor kicks in, a loose jumper). Marking it lost hands it back
        // to the retry path above, so it recovers without a reboot. (The gyro
        // has its own watchdog inside BOTGyro.) ---
        if (now - lastHealthCheck >= healthCheckInterval) {
            lastHealthCheck = now;
            if (display.isReady() && !display.isResponding()) {
                DBG_PRINTLN("Display stopped responding, re-initialising");
                display.markLost();
            }
        }

        // --- Update Display (every DISPLAY_REFRESH_INTERVAL_MS) ---
        if (display.isReady() && now - lastDisplayUpdate >= displayInterval) {
            lastDisplayUpdate = now;

            // Copy the numbers out under the lock and format them after it is
            // released: loop() only ever tries the lock, so holding it while
            // building text would make it skip its exchange for nothing.
            SharedData snap;
            bool runTimeStale = false;
            if (xSemaphoreTake(sharedMutex, portMAX_DELAY) == pdTRUE) {
                snap = shared;
                // the run clock is the Pi's, we only draw it. the stale flag is
                // worked out from the same copy so the line can never mix a
                // fresh time with a stale flag
                runTimeStale = shared.runTimeSeen &&
                               (millis() - shared.runTimeRxMs > RUN_TIMER_STALE_MS);
                xSemaphoreGive(sharedMutex);
            }

            // 8 lines of 21 characters (6x8 px font)
            const size_t LEN = 22;
            char line[LEN];
            char imu1[LEN], imu2[LEN];
            imuLines(snap, imu1, imu2, LEN);

            display.clear();
            snprintf(line, LEN, "Mot %+4d%% Str %3u", snap.motorPercent, (unsigned)snap.steeringAngle);
            display.displayText(line, 0, 0, 1);
            display.displayText(driveStateLine(snap.driveState), 0, 8, 1);
            snprintf(line, LEN, "Enc %ld", snap.encoderCount);
            display.displayText(line, 0, 16, 1);
            snprintf(line, LEN, "Yaw %+7.1f", snap.heading);
            display.displayText(line, 0, 24, 1);
            snprintf(line, LEN, "P %+6.1f R %+6.1f", snap.pitchDeg, snap.rollDeg);
            display.displayText(line, 0, 32, 1);
            display.displayText(imu1, 0, 40, 1);
            display.displayText(imu2, 0, 48, 1);
            display.displayText(runTimerLine(snap.runTimeMs, snap.runTimeState, runTimeStale), 0, 56, 1);
            display.updateScreen();    // single I2C transfer
        }

        vTaskDelay(1);  // Yield to other tasks (small delay)
    }
}


void setup(){
    // First thing, before anything attaches a PWM pin: hand all four LEDC
    // timers to ESP32PWM. It then gives each frequency its own timer, so the
    // two 20 kHz motor channels share one and the 50 Hz servo can never be
    // put on it (ESP32Servo 3.x actually drives the servo from MCPWM on the
    // S3, which keeps the two apart regardless).
    for (int timer = 0; timer < LEDC_TIMER_COUNT; timer++) {
        ESP32PWM::allocateTimer(timer);
    }
    // the bridge comes up braked and stays braked until a valid drive command
    motor.begin(MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS,
                MOTOR_SLEW_PERCENT_PER_S);

    debugSerialBegin();          // native USB, human readable logs only
    // MAVLink to/from the Pi over its GPIO UART. Buffer sizes have to be set
    // before begin(), the core refuses to resize a running port.
    PI_SERIAL.setRxBufferSize(PI_UART_RX_BUFFER_BYTES);
    PI_SERIAL.setTxBufferSize(PI_UART_TX_BUFFER_BYTES);
    PI_SERIAL.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
    // centred straight away, and held there until a valid drive command
    if (!steer.begin(SERVO_FREQUENCY_HZ, STEERING_MIN_US, STEERING_CENTER_US, STEERING_MAX_US)) {
        statusTextf(STATUSTEXT_SEVERITY_ERROR, "Steering servo failed to attach");
    }
    encoder.begin(ENCODER_SAMPLE_INTERVAL_MS);
    button.begin();
    // stays off until the ROS2 bridge announces itself over serial, see the
    // connect message handling in loop()
    status_led.begin();
    // same link state in colour, plus what the drivetrain is doing. lights red
    // straight away, so it is obvious the firmware got this far even when
    // nothing is connected yet.
    rgb_led.begin(RGB_STATUS_LED_BRIGHTNESS, RGB_STATUS_FLASH_PERIOD_MS,
                  STEERING_CENTER_ANGLE, RGB_STEERING_CENTER_TOLERANCE_DEG);
    // The I2C bus for the BOTGyro and the OLED. Pins first, then begin(), both
    // before the I2C task (and so before gyro.begin(), whose own Wire.begin()
    // is then a no-op on the right pins). The OLED driver is told not to call
    // Wire.begin() at all (periphBegin=false).
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    sharedMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(i2cTask,"I2C Task", 8192, NULL, 1, &i2cTaskHandle, 0);
}


// --- IMU, as loop() sees it (copied out of SharedData each pass) -----------
struct ImuView {
    uint32_t i2cClockHz = I2C_CLOCK_HZ;
    ImuStatus status = ImuStatus::NotFound;
    bool beginTried = false;
    bool everCalibrated = false;
    uint16_t sampleRateHz = 0;
    float yawDeg = 0.0f;
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
};
static ImuView imu;

// Drive commands are ignored until the first IMU calibration, or until
// IMU_CAL_GATE_MAX_MS after boot, whichever comes first. Once open it stays
// open: a gyro that faults or recalibrates mid run never stops the car.
static bool imuGateOpen = false;
RateLimitedLog imuHoldLog(DRIVE_LOG_MIN_INTERVAL_MS);

static void updateImuGate(uint32_t nowMs) {
    if (imuGateOpen) return;
    if (imu.everCalibrated) {
        imuGateOpen = true;
        statusTextf(STATUSTEXT_SEVERITY_INFO, "Drive: accepted, IMU calibrated after %lu ms",
                    (unsigned long)nowMs);
    } else if (nowMs >= IMU_CAL_GATE_MAX_MS) {
        imuGateOpen = true;
        statusTextf(STATUSTEXT_SEVERITY_WARNING, "Drive: accepted after %lu ms, IMU %s",
                    (unsigned long)nowMs, imuStatusName(imu.status));
    }
}

// The plain LED shows the IMU while it needs attention (the start countdown
// still outranks it, and the link level comes back when it is fine).
static void updateStatusLedAlert() {
    switch (imu.status) {
        case ImuStatus::Calibrating:
            status_led.setAlert(STATUS_LED_CAL_PERIOD_MS, STATUS_LED_CAL_PERIOD_MS / 2);
            break;
        case ImuStatus::NotFound:
            // nothing to say until the first attempt has actually failed
            if (imu.beginTried) {
                status_led.setAlert(STATUS_LED_FAULT_PERIOD_MS, STATUS_LED_FAULT_ON_MS);
            }
            break;
        case ImuStatus::Fault:
        case ImuStatus::Uncalibrated:
            status_led.setAlert(STATUS_LED_FAULT_PERIOD_MS, STATUS_LED_FAULT_ON_MS);
            break;
        case ImuStatus::Ready:
        default:
            status_led.setAlert(0, 0);
            break;
    }
}

// Tells the Pi (MAVLink STATUSTEXT, logged by mcu_bridge) when the IMU state
// changes, and keeps repeating it while the IMU is not usable. Also reports
// the I2C clock fallback, once.
static void reportImuStatus(uint32_t nowMs) {
    static bool reportedOnce = false;
    static ImuStatus lastReported = ImuStatus::NotFound;
    static uint32_t lastReportMs = 0;

    static bool fallbackReported = false;
    if (!fallbackReported && imu.i2cClockHz != I2C_CLOCK_HZ) {
        char text[MAVLINK_STATUSTEXT_TEXT_LEN + 1];
        snprintf(text, sizeof(text), "I2C unreliable at %lu kHz, bus now %lu kHz",
                 (unsigned long)(I2C_CLOCK_HZ / 1000), (unsigned long)(imu.i2cClockHz / 1000));
        statusTextf(STATUSTEXT_SEVERITY_WARNING, "%s", text);
        fallbackReported = true;
    }

    if (!imu.beginTried) return;  // the first begin() is still running
    bool unusable = imu.status == ImuStatus::NotFound || imu.status == ImuStatus::Fault ||
                    imu.status == ImuStatus::Uncalibrated;
    bool changed = !reportedOnce || imu.status != lastReported;
    bool repeatDue = unusable && (uint32_t)(nowMs - lastReportMs) >= IMU_STATUSTEXT_REPEAT_MS;
    if (!changed && !repeatDue) return;

    char text[MAVLINK_STATUSTEXT_TEXT_LEN + 1];
    uint8_t severity;
    switch (imu.status) {
        case ImuStatus::Ready:
            severity = STATUSTEXT_SEVERITY_INFO;
            snprintf(text, sizeof(text), "IMU ready, %u Hz", (unsigned)imu.sampleRateHz);
            break;
        case ImuStatus::Calibrating:
            severity = STATUSTEXT_SEVERITY_NOTICE;
            snprintf(text, sizeof(text), "IMU calibrating, keep the car still");
            break;
        case ImuStatus::Uncalibrated:
            severity = STATUSTEXT_SEVERITY_WARNING;
            snprintf(text, sizeof(text), "IMU not calibrated, yaw held");
            break;
        case ImuStatus::Fault:
            severity = STATUSTEXT_SEVERITY_ERROR;
            snprintf(text, sizeof(text), "IMU fault: gyro stopped answering, retrying");
            break;
        case ImuStatus::NotFound:
        default:
            severity = STATUSTEXT_SEVERITY_ERROR;
            snprintf(text, sizeof(text), "IMU not found at 0x%02X, retrying", IMU_ADDRESS);
            break;
    }
    statusTextf(severity, "%s", text);
    reportedOnce = true;
    lastReported = imu.status;
    lastReportMs = nowMs;
}

static const char *driveStateName() {
    if (!imuGateOpen) return "IMU-HOLD";
    if (driveActive) return "ACTIVE";
    return driveWatchdog.everFed() ? "STOPPED" : "WAITING";
}

// DEBUG_PRINTS builds only: one line a second, with the longest gap between
// two loop() passes since the previous line (the drive timeout check runs
// once per pass, so that gap is how late it can be).
static void debugStatusLine(uint32_t maxLoopGapUs) {
    DBG_PRINTF("[status] drive=%s mot=%+d%% str=%u | imu=%s %uHz yaw=%+.1f pitch=%+.1f roll=%+.1f | loop gap max %lu us\n",
                        driveStateName(), motor.outputPercent(), (unsigned)steer.getAngle(),
                        imuStatusName(imu.status), (unsigned)imu.sampleRateHz,
                        imu.yawDeg, imu.pitchDeg, imu.rollDeg, (unsigned long)maxLoopGapUs);
}

// Run timer frames land in loop() but belong on the OLED. They are parked
// here and handed over at the next shared-data exchange, so a busy lock delays
// one by a pass instead of dropping it.
struct PendingRunTimer {
    bool pending = false;
    uint32_t elapsedMs = 0;
    uint8_t state = 0;
    unsigned long rxMs = 0;
};
static PendingRunTimer pendingRunTimer;

void loop(){
    // how long since the previous pass: the drive timeout below is checked
    // once per pass, so this is how late it can ever be
    static uint32_t lastLoopUs = micros();
    static uint32_t maxLoopGapUs = 0;
    uint32_t loopUs = micros();
    uint32_t loopGapUs = loopUs - lastLoopUs;
    lastLoopUs = loopUs;
    if (loopGapUs > maxLoopGapUs) maxLoopGapUs = loopGapUs;

    encoder.update();

    // --- Shared data with the I2C task. NEVER waits: the other core only
    // holds the lock long enough to copy a few numbers, and if it holds it
    // right now this pass skips the exchange and the next one (microseconds
    // later) does it. Values read here keep their last good copy ---
    static float headingDeg = 0.0f;
    if (xSemaphoreTake(sharedMutex, 0) == pdTRUE) {
        shared.motorPercent = motor.outputPercent();
        shared.steeringAngle = steer.getAngle();
        shared.encoderCount = encoder.count();
        shared.driveState = !imuGateOpen ? DRIVE_IMU_HOLD
                          : driveActive ? DRIVE_ACTIVE
                          : (driveWatchdog.everFed() ? DRIVE_STOPPED : DRIVE_WAITING);
        if (pendingRunTimer.pending) {
            shared.runTimeMs = pendingRunTimer.elapsedMs;
            shared.runTimeState = pendingRunTimer.state;
            shared.runTimeSeen = true;
            shared.runTimeRxMs = pendingRunTimer.rxMs;
            pendingRunTimer.pending = false;
        }
        // written by the I2C task on the other core
        headingDeg = shared.heading;
        imu.status = (ImuStatus)shared.imuStatus;
        if (shared.i2cClockHz != 0) imu.i2cClockHz = shared.i2cClockHz;
        imu.beginTried = shared.imuBeginTried;
        imu.everCalibrated = shared.imuEverCalibrated;
        imu.sampleRateHz = shared.imuSampleRateHz;
        imu.yawDeg = shared.heading;
        imu.pitchDeg = shared.pitchDeg;
        imu.rollDeg = shared.rollDeg;
        xSemaphoreGive(sharedMutex);
    }

    // --- IMU: open the drive gate once calibrated (or after the time limit),
    // show it on the LED, tell the Pi about changes ---
    updateImuGate(millis());
    updateStatusLedAlert();
    reportImuStatus(millis());


    // --- Button: poll every loop, but telemetry only goes out every
    // SENSOR_TX_INTERVAL_MS. Latch the press edge so a tap shorter than that
    // interval still reaches ROS2 instead of falling between two frames ---
    static bool buttonPressLatch = false;
    if (button.isPressed()) {
        buttonPressLatch = true;
        DBG_PRINTLN("Start (BOOT) button pressed");
        // count the start delay out on the LED. ROS2 starts its own timer off
        // the same press, so both ends run the same countdown - the blinks are
        // over at the moment the car begins to move.
        status_led.blink(START_BLINK_COUNT, START_BLINK_INTERVAL_MS);
    }
    // advances the countdown blinks and the IMU pattern
    status_led.update();

    // --- RGB pixel: read the drivetrain back out every pass rather than
    // latching what the last serial frame asked for, so it shows what the motor
    // and servo are actually holding. update() only touches the pixel when the
    // colour changes, so this is free on the quiet frames ---
    rgb_led.setDrive(motor.outputPercent(), steer.getAngle());
    rgb_led.update();

    // --- Send encoder/steering/heading/button telemetry to ROS2 ---
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

        // yaw in degrees, (-180, 180], clockwise positive, the same value the
        // OLED prints. Relative to the heading at boot and slowly drifting
        // (integrated gyro, no magnetometer); the ROS nodes only ever use
        // wrapped differences of it, so the BNO055's 0..360 range is not needed.
        float headingField = headingDeg;

        // held down right now, or tapped since the last frame went out
        uint8_t buttonField = (button.isDown() || buttonPressLatch) ? 1 : 0;
        buttonPressLatch = false;

        mavlink_message_t txMsg;
        uint8_t txBuf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack(system_id, component_id, &txMsg,
            encoderCountField, encoderSpeedField, encoderDirectionField, steer.getAngle(),
            headingField,
            // no sonars fitted, see config.h
            SONAR_WIRE_NO_READING_CM, SONAR_WIRE_NO_READING_CM,
            SONAR_WIRE_NO_READING_CM, SONAR_WIRE_NO_READING_CM,
            buttonField);
        uint16_t txLen = mavlink_msg_to_send_buffer(txBuf, &txMsg);
        PI_SERIAL.write(txBuf, txLen);
    }


    while(PI_SERIAL.available()>0){
        uint8_t c = PI_SERIAL.read();
        
        mavlink_message_t received_msg;
        mavlink_status_t status;

        // only MAVLINK_FRAMING_OK is a whole frame with a good CRC; bad CRCs
        // and half frames come back as something else and are dropped here
        uint8_t framing = mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &status);
        if(framing == MAVLINK_FRAMING_OK){
            DriveCommand driveCmd;
            // the only frame that may refresh the drive watchdog
            if(decodeDriveCommand(framing, received_msg, driveCmd)){
                if (imuGateOpen) {
                    applyDriveCommand(driveCmd, millis());
                } else if (imuHoldLog.shouldLog(millis())) {
                    // ignored: no watchdog refresh, the motor stays braked
                    statusTextf(STATUSTEXT_SEVERITY_NOTICE, "Drive: ignored until IMU calibrated, keep still");
                }
            }
            // mcu_bridge sends this once, right after it opens the port. it is the
            // only announcement we get that a ROS2 side actually exists, so it is
            // what drives the status LED. if a start countdown happens to be
            // blinking, set() just records the level and the LED lands on it once
            // the countdown finishes.
            else if(received_msg.msgid == MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg){
                uint8_t connected = mavlink_msg_gorur_gari_ros2_to_mcu_connect_msg_get_connected(&received_msg);
                status_led.set(connected != 0);
                // the pixel latches it too - it cannot read the plain LED back,
                // since that pin belongs to the countdown while one is running
                rgb_led.setConnected(connected != 0);
                DBG_PRINTLN(connected ? "ROS2 bridge connected" : "ROS2 bridge disconnected");
            }
            // the run stopwatch, sent ten times a second by mcu_bridge while
            // autonomy/run_timer is up. it is display only: nothing in here
            // steers, brakes or counts on it, so frames going missing costs a
            // line on the screen and nothing else
            else if(received_msg.msgid == MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_timer_msg){
                // parked, handed to the I2C task at the next exchange
                pendingRunTimer.elapsedMs = mavlink_msg_gorur_gari_ros2_to_mcu_timer_msg_get_elapsed_ms(&received_msg);
                pendingRunTimer.state = mavlink_msg_gorur_gari_ros2_to_mcu_timer_msg_get_state(&received_msg);
                pendingRunTimer.rxMs = millis();
                pendingRunTimer.pending = true;
            }
        }
    }

    // --- Drive safety: brake + centre if the valid commands have stopped.
    // Every pass, after the serial input, so a command that landed in this
    // pass counts. Nothing above can block for long: no delay(), no I2C (that
    // is all on the other core), and serial writes go into a ring buffer ---
    enforceDriveTimeout(millis());

    // --- Motor: advance the slew ramp towards whatever the last command asked
    // for, or hold the brake ---
    motor.update();

    // --- debug status line (DEBUG_PRINTS builds only) ---
    static uint32_t lastStatusMs = 0;
    uint32_t statusNowMs = millis();
    if (DEBUG_PRINTS && DEBUG_STATUS_INTERVAL_MS > 0 &&
        statusNowMs - lastStatusMs >= DEBUG_STATUS_INTERVAL_MS) {
        lastStatusMs = statusNowMs;
        debugStatusLine(maxLoopGapUs);
        maxLoopGapUs = 0;
    }
}



