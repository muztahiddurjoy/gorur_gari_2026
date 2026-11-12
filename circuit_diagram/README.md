# Circuit diagrams

This folder holds block diagrams of the gorur_gari_2026 electronics, generated from the wiring the firmware actually uses.

| File | Contents |
| --- | --- |
| `circuit_block_diagram.png` / `.svg` | Full system block diagram: Pi 4B, ESP32-S3, sensors, actuators, and signal buses with GPIO labels, plus a key for the WS2812 status colours |
| `esp32_pin_map.png` | ESP32-S3 pin assignment table (used and reserved pins) |
| `generate_diagrams.py` | Generator script (matplotlib) |

## How the boards are wired

```mermaid
flowchart LR
    PI["Raspberry Pi 4B\n/dev/gorur_mcu"] <-->|"USB cable, devkit 'USB' port\nnative USB CDC, MAVLink 2"| ESP["ESP32-S3-DevKitC-1 N16R8"]
    ESP -->|"RPWM 5, LPWM 6, EN 7"| DRV["BTS7960 (IBT-2)"] --> MOT["Drive motor"]
    ENC["Encoder C1 1 / C2 2"] --> ESP
    ESP -->|"PWM 16, 50 Hz"| SERVO["MG996R servo (own 6 V buck)"]
    ESP <-->|"I2C SDA 8 / SCL 9"| I2C["BOTGyro 0x6A + OLED 0x3C"]
    BTN["START = BOOT button (GPIO 0)"] --> ESP
    ESP --> LED["Status LED (GPIO 4)"]
    ESP --> PIX["Onboard WS2812 (GPIO 48)"]
```

The Pi and the ESP32-S3 talk over one USB cable into the devkit's port labelled "USB" (the S3's native USB-Serial/JTAG, VID 303a, `/dev/gorur_mcu` on the Pi via `ros2_ws/config/99-gorur.rules`). It carries MAVLink only. GPIO40/41, the old UART link, are unused. There are no sonars on this hardware revision. GPIO35-37 belong to the N16R8 module's octal PSRAM and are never used. The encoder phases are ordered so forward motion counts as positive, see `pins.h`.

## Status lights

Two of the blocks need no wiring decisions but do need explaining, since they are the only output the car has once it is on the mat.

The **status LED on GPIO 4** is a plain single-colour LED (GPIO → 220–470 Ω → anode, cathode to GND). It is off from boot, lights when the ROS 2 bridge announces itself, and blinks the 5-second start countdown after a press of the BOOT button (GPIO 0, the start button). While the IMU calibrates it blinks fast (keep the car still); if the IMU is missing or faulted it gives a short blip once a second.

The **WS2812 on GPIO 48** is soldered to the devkit, so it costs no wiring and no extra pin. It repeats the link state in colour and adds what the drivetrain is doing:

| Colour | Meaning |
| --- | --- |
| Red | No ROS 2 bridge has announced itself since boot |
| Green | Bridge connected, drivetrain idle |
| Flashing blue | Motor driving, steering within a couple of degrees of centre |
| Flashing purple | Motor driving with the wheels turned |

Driving outranks the link colour, so a car that is moving never shows a resting colour. Driven by `firmware/lib/rgb_status_led` through the Arduino core's RGB LED call (`neopixelWrite()` / `rgbLedWrite()` on core 3, RMT bit banging, no pixel library); brightness, flash period and the "centred" tolerance are in `firmware/include/config.h`.

## Regenerating the diagrams

The diagrams are generated, not hand-drawn, so re-run the script whenever the wiring changes:

```sh
python3 generate_diagrams.py   # needs matplotlib
# python3 -s generate_diagrams.py if a NumPy 2 in ~/.local breaks the system matplotlib
```

## Sources of truth

These are the files the generator actually reads. If wiring changes, it starts in one of these:

| File | What it defines |
| --- | --- |
| `firmware/include/pins.h` | GPIO assignments |
| `firmware/include/config.h` | PWM frequencies, I²C addresses, timeouts and limits |
| `firmware/pin-map.md` | Full pin table, including reserved slots |
| `firmware/lib/rgb_status_led/` | What each WS2812 colour means |
| `ros2_ws/controls/controls/mcu_bridge.py` | The Pi-to-MCU serial link |
