# ESP32-S3 pin map

Board: **ESP32-S3-DevKitC-1 N16R8**. The octal PSRAM on this module uses **GPIO35-37**, so none of them may be used. Every pin below is defined in [`include/pins.h`](./include/pins.h); every timing and limit is in [`include/config.h`](./include/config.h).

## Overview by subsystem

```mermaid
flowchart LR
    subgraph I2C["I2C bus (SDA 8 / SCL 9, 400 kHz)"]
        IMU["BOTGyro IMU\n0x6A"]
        OLED["SSD1306 OLED\n0x3C"]
    end
    subgraph DRIVE["Drive train"]
        MOTOR["BTS7960 (IBT-2)\nRPWM 5, LPWM 6, R_EN+L_EN 7"]
        SERVO["MG996R servo: PWM 16\n(own 6 V buck)"]
        ENC["Encoder motor\nC1 1, C2 2"]
    end
    subgraph LINK["Raspberry Pi link"]
        USB["devkit 'USB' port (native USB CDC)\nPi: /dev/gorur_mcu"]
    end
    subgraph IO["Button and status lights"]
        B1["Start = BOOT button: GPIO 0"]
        L1["Status LED: GPIO 4"]
        L3["Onboard WS2812: GPIO 48"]
    end
```

## Full table

| Function | GPIO | Notes |
| --- | --- | --- |
| BTS7960 RPWM (forward PWM) | **5** | 20 kHz, 8-bit; 10k pull-down on the board |
| BTS7960 LPWM (reverse PWM) | **6** | 20 kHz, 8-bit; 10k pull-down on the board |
| BTS7960 R_EN + L_EN (tied) | **7** | 10k pull-down on the board |
| Steering servo (MG996R) signal | **16** | 50 Hz; the servo is powered from a separate 6 V buck, only signal + ground come from the ESP32 |
| Encoder C1 / C2 | **1 / 2** | quadrature, 3.3 V (A and B deliberately swapped vs. the silkscreen, see `pins.h`) |
| I2C SDA / SCL | **8 / 9** | `Wire.setPins(8, 9)` before `Wire.begin()`; BOTGyro 0x6A (0x6B if SA0 high), SSD1306 OLED 0x3C |
| Start button = the devkit's BOOT button | **0** | INPUT_PULLUP, active low, 30 ms software debounce |
| Status LED | **4** | plain LED, GPIO -> 220-470 R -> anode, cathode to GND |
| Link to the Raspberry Pi | **native USB (19 / 20)** | the devkit's "USB" port (USB-Serial/JTAG, VID 303a), USB CDC = `Serial` = `MAV_SERIAL`, MAVLink only; Pi side `/dev/gorur_mcu` |
| Unused | **40, 41** | the old UART link to the Pi, deliberately left free |
| Onboard WS2812 status pixel | **48** | soldered to the devkit (`RGB_BUILTIN`) |

GPIO35, 36 and 37 are the octal PSRAM's. GPIO19/20 are the native USB data lines of the Pi link and must not be used for anything else.

## Notes

### Motor driver (BTS7960)

`MotorController` (`lib/motor_control`) drives RPWM and LPWM as two ESP32PWM (LEDC) channels at 20 kHz and the tied enables as a plain output:

| State | RPWM | LPWM | EN |
| --- | --- | --- | --- |
| Forward | duty | 0 (written first) | HIGH |
| Reverse | 0 (written first) | duty | HIGH |
| Brake | 0 | 0 | HIGH |
| Coast | 0 | 0 | LOW |

The two PWM pins are never both non-zero. The output is ramped at `MOTOR_SLEW_PERCENT_PER_S`, and a change of direction always passes through a brake. The safety stop bypasses the ramp. The int8 throttle on the wire maps +/-127 to full duty.

### Steering servo

The 0-180 steering command (90 = straight) is mapped onto `STEERING_MIN_US` / `STEERING_CENTER_US` / `STEERING_MAX_US`, and nothing outside MIN..MAX is ever sent. **Calibrate those three on the bench before driving.** The defaults are a conservative +/-400 us around 1500 us.

### PWM timers

`setup()` calls `ESP32PWM::allocateTimer(0..3)` before anything attaches, so ESP32PWM gives each frequency its own LEDC timer: the two 20 kHz motor channels share one, and the 50 Hz servo never lands on it. ESP32Servo 3.x actually drives the servo from MCPWM on the S3, which keeps them apart in any case.

### Start button on GPIO0

GPIO0 is a strapping pin. The firmware only reads it, after boot, with the internal pull-up, and never drives it. Nothing else may be wired to it. Holding BOOT through a reset still enters the ROM bootloader, as on any devkit.

### Status LED (GPIO4)

Priority order:

1. The start countdown: after a start button press, five blinks, once a second (5 s), then back to the link level as the car moves.
2. IMU patterns: a fast 5 Hz blink while the gyro calibrates (keep the car still), or a short blip once a second if the IMU is missing, faulted or uncalibrated.
3. The link level: lit once the ROS 2 bridge has announced itself, then whatever `mcu_bridge`'s `/status_led` topics ask for.

### Onboard WS2812 (GPIO48)

It is soldered to the devkit, so it costs no wiring. It repeats the link state in colour and adds what the drivetrain is doing, which the single plain LED cannot show:

| Colour              | Meaning                                                    |
| ------------------- | ---------------------------------------------------------- |
| Red                 | No ROS2 bridge has announced itself since boot              |
| Green               | Bridge connected, drivetrain idle                           |
| Flashing blue       | Motor driving, steering within a couple of degrees of centre |
| Flashing purple     | Motor driving with the wheels turned                        |

Driving outranks the link colour, because wheels turning is the thing worth spotting from across the table and the link state is on the plain LED anyway. Reverse counts as driving. Brightness, flash period and the "centred" tolerance are in `firmware/include/config.h`. The driver is `firmware/lib/rgb_status_led`, which uses the Arduino core's RGB LED call (RMT) rather than a pixel library.

Both lights follow the same one-shot connect message, so read red as "never connected since boot" rather than "not connected right now". The OLED's `Cmd:` line shows whether drive commands are arriving *now*.

Some ESP32-S3-DevKitC-1 revisions route the pixel to a different GPIO. `RGB_STATUS_LED_PIN` in `firmware/include/pins.h` follows the board variant's `RGB_BUILTIN` and is the one place to change it.

### Link to the Pi

A USB cable from the Pi into the devkit's port labelled **"USB"**: the S3's own USB-Serial/JTAG controller, native USB CDC (`ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1`), Espressif VID 303a. Not the "UART"/"COM" port, whose USB-UART bridge chip clashed with the LiDAR's CP2102 in udev. On the Pi, `ros2_ws/config/99-gorur.rules` names it `/dev/gorur_mcu`.

The port carries MAVLink 2 and nothing else. Debug text is compiled out (`DEBUG_PRINTS=0`), and events go to the Pi as STATUSTEXT. Nothing waits for the host: the car boots, runs the IMU and enforces the drive timeout with or without the Pi, and frames that do not fit the TX buffer are dropped whole.

The car moves only while valid drive commands keep arriving. After 100 ms without one (`CMD_TIMEOUT_MS`), or immediately when the USB host disappears (start-of-frame packets stop: cable pulled, Pi off), the motor brakes and the steering centres. `mcu_bridge` sends drive commands at 50 Hz while `/cmd_vel` is fresh.
