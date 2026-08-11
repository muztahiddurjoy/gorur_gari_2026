# Circuit diagrams

Block diagrams of the gorur_gari_2026 electronics, generated from the wiring
that the firmware actually uses:

| File | Contents |
| --- | --- |
| `circuit_block_diagram.png` / `.svg` | Full system block diagram — Pi 4B, ESP32-S3, sensors, actuators, signal buses with GPIO labels |
| `esp32_pin_map.png` | ESP32-S3 pin assignment table (used + reserved pins) |
| `generate_diagrams.py` | Generator script (matplotlib) |

Sources of truth — regenerate after changing any of these:

- `firmware/include/pins.h` — GPIO assignments
- `firmware/include/config.h` — PWM frequencies, I²C addresses, sonar enables
- `firmware/pin-map.md` — full pin table incl. reserved slots
- `ros2_ws/controls/controls/mcu_bridge.py` — Pi ↔ MCU serial link

Regenerate with:

```sh
python3 generate_diagrams.py   # needs matplotlib
```

Notes reflected in the diagram:

- The 3 fitted HC-SR04s are FRONT/LEFT/RIGHT; the REAR slot (GPIO40/41) is
  reserved but unpopulated. All sonars are currently disabled in `config.h`.
- Encoder A/B are intentionally swapped relative to the silkscreen so forward
  motion counts positive (`pins.h`).
- The Pi talks to the ESP32-S3 over native USB CDC (`/dev/esp32_s3`,
  MAVLink 2 @ 115200) — no GPIO-level link between them.
