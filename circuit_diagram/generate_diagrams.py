#!/usr/bin/env python3
"""Generate the circuit block diagram images for gorur_gari_2026.

Wiring is taken from the firmware sources — keep in sync with:
  firmware/include/pins.h      (GPIO assignments)
  firmware/include/config.h    (frequencies, I2C addresses, enables)
  firmware/pin-map.md          (full pin table incl. reserved slots)
  firmware/lib/rgb_status_led/ (what each WS2812 colour means)
  ros2_ws/controls/controls/mcu_bridge.py  (/dev/esp32_s3 MAVLink link)

Usage:  python3 generate_diagrams.py   (needs matplotlib)
Outputs circuit_block_diagram.png/.svg and esp32_pin_map.png next to it.
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- palette
C_COMPUTE = dict(fc="#dbeafe", ec="#1d4ed8")   # Pi + USB peripherals
C_MCU     = dict(fc="#fef3c7", ec="#b45309")   # ESP32-S3
C_SENSOR  = dict(fc="#dcfce7", ec="#15803d")   # sonar, IMU, encoder
C_ACT     = dict(fc="#fee2e2", ec="#b91c1c")   # driver, motor, servo
C_HMI     = dict(fc="#f3e8ff", ec="#7e22ce")   # OLED, button, LED

W_USB   = "#1d4ed8"
W_I2C   = "#7e22ce"
W_PWM   = "#ea580c"
W_GPIO  = "#15803d"
W_ENC   = "#0d9488"
W_PHASE = "#991b1b"

# what the onboard WS2812 shows, see firmware/lib/rgb_status_led
PIXEL_STATES = [
    ("#dc2626", "red",     "no ROS 2 bridge since boot"),
    ("#16a34a", "green",   "bridge connected, car idle"),
    ("#2563eb", "blue",    "driving, steering centred (flashing)"),
    ("#a21caf", "purple",  "driving, wheels turned (flashing)"),
]

FIG_BG = "white"


def block(ax, x, y, w, h, title, sub=None, style=C_SENSOR, tfs=10, sfs=7.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.4,rounding_size=1.2",
                                fc=style["fc"], ec=style["ec"], lw=1.6, zorder=3))
    cy = y + h / 2
    if sub:
        ax.text(x + w / 2, cy + h * 0.14, title, ha="center", va="center",
                fontsize=tfs, fontweight="bold", color="#111827", zorder=4)
        ax.text(x + w / 2, cy - h * 0.22, sub, ha="center", va="center",
                fontsize=sfs, color="#374151", zorder=4)
    else:
        ax.text(x + w / 2, cy, title, ha="center", va="center",
                fontsize=tfs, fontweight="bold", color="#111827", zorder=4)


def wire(ax, pts, color, lw=1.8, ls="-", arrow=None, zorder=2):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=zorder,
            solid_capstyle="round", solid_joinstyle="round")
    for end in (arrow or ""):
        if end == ">":
            ax.annotate("", xy=pts[-1], xytext=pts[-2],
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw))
        if end == "<":
            ax.annotate("", xy=pts[0], xytext=pts[1],
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw))


def wlabel(ax, x, y, text, color, fs=6.8, ha="center"):
    ax.text(x, y, text, ha=ha, va="center", fontsize=fs, color=color,
            zorder=5, bbox=dict(boxstyle="round,pad=0.18", fc="white",
                                ec="none", alpha=0.92))


def dot(ax, x, y, color):
    ax.plot([x], [y], marker="o", ms=4, color=color, zorder=5)


# ================================================================ figure 1
def main_diagram():
    fig, ax = plt.subplots(figsize=(17, 11), dpi=200)
    fig.patch.set_facecolor(FIG_BG)
    ax.set_xlim(0, 170)
    ax.set_ylim(0, 108)
    ax.axis("off")

    ax.text(85, 105.2, "GORUR GARI 2026 — SYSTEM CIRCUIT BLOCK DIAGRAM",
            ha="center", va="center", fontsize=15, fontweight="bold",
            color="#111827")
    ax.text(85, 101.8,
            "wiring per firmware/include/pins.h · firmware/include/config.h · firmware/pin-map.md",
            ha="center", va="center", fontsize=8, color="#6b7280")

    # ---------------- compute tier -------------------------------------
    block(ax, 8, 86, 32, 12, "RPLidar C1",
          "360° 2D LiDAR\nUSB (UART adapter)", C_COMPUTE)
    block(ax, 62, 84, 46, 15, "Raspberry Pi 4B",
          "ROS 2 host — ros2_ws/\nlidar + camera + planner nodes", C_COMPUTE, tfs=12)
    block(ax, 130, 86, 32, 12, "Logitech C922 Pro",
          "webcam, UVC\nUSB 2.0", C_COMPUTE)

    wire(ax, [(40, 92), (62, 92)], W_USB, arrow="<>")
    wlabel(ax, 51, 94.2, "USB", W_USB)
    wire(ax, [(130, 92), (108, 92)], W_USB, arrow="<>")
    wlabel(ax, 119, 94.2, "USB", W_USB)

    # Pi <-> ESP32 (USB-to-TTL adapter into UART1, MAVLink)
    wire(ax, [(85, 84), (85, 73)], W_USB, lw=2.4, arrow="<>")
    wlabel(ax, 86, 78.5,
           "USB-to-TTL → UART1 (RX 18 / TX 17)  ·  MAVLink 2 @ 115200\n"
           "/dev/esp32_s3  ·  mcu_bridge.py  ·  telemetry every 50 ms",
           W_USB, fs=7, ha="left")

    # ---------------- MCU ----------------------------------------------
    block(ax, 58, 42, 54, 31, "", style=C_MCU)
    ax.text(85, 68.5, "ESP32-S3 DevKitC-1", ha="center", fontsize=13,
            fontweight="bold", color="#111827", zorder=4)
    ax.text(85, 65,
            "firmware/ — PlatformIO · Arduino · FreeRTOS\n"
            "core 1: loop() — motor, servo, sonar, encoder, status lights, MAVLink\n"
            "core 0: i2cTask — BNO055 @ 10 ms, OLED @ 50 ms",
            ha="center", va="top", fontsize=7.5, color="#374151", zorder=4)

    # ---------------- I2C bus (left) ------------------------------------
    block(ax, 8, 58, 32, 12, "BNO055 IMU",
          "9-DoF fusion, yaw\nI²C addr 0x29", C_SENSOR)
    block(ax, 8, 42, 32, 12, "OLED SSD1306",
          "128×64 status display\nI²C addr 0x3C", C_HMI)

    trunk_x = 49
    wire(ax, [(58, 56), (trunk_x, 56)], W_I2C, lw=2.2)          # ESP32 -> trunk
    wire(ax, [(trunk_x, 64), (trunk_x, 48)], W_I2C, lw=2.2)     # trunk
    wire(ax, [(trunk_x, 64), (40, 64)], W_I2C, lw=2.2, arrow=">")
    wire(ax, [(trunk_x, 48), (40, 48)], W_I2C, lw=2.2, arrow=">")
    dot(ax, trunk_x, 56, W_I2C)
    dot(ax, 58, 56, W_I2C)
    wlabel(ax, 49, 39.4, "I²C bus (3.3 V)\nSDA = GPIO8 · SCL = GPIO9", W_I2C, fs=7)

    # ---------------- motor chain (right) --------------------------------
    block(ax, 128, 55, 34, 17, "TB6612FNG",
          "H-bridge motor driver\nch A · VM = VBAT · VCC = 3V3", C_ACT)
    block(ax, 128, 31, 34, 12, "JGA370 motor",
          "DC gearmotor + quadrature\nencoder (1320 cnt/rev)", C_SENSOR)

    ctrl = [("GPIO4 → PWMA · 20 kHz", 69.5, W_PWM),
            ("GPIO5  → AIN1", 66.0, W_GPIO),
            ("GPIO6  → AIN2", 62.5, W_GPIO),
            ("GPIO7  → STBY", 59.0, W_GPIO)]
    for label, yy, cc in ctrl:
        wire(ax, [(112, yy), (128, yy)], cc, arrow=">")
        wlabel(ax, 120, yy + 1.5, label, cc, fs=6.4)

    # motor phases driver -> motor
    wire(ax, [(136, 55), (136, 43)], W_PHASE, lw=3.2, arrow=">")
    wire(ax, [(141, 55), (141, 43)], W_PHASE, lw=3.2, arrow=">")
    wlabel(ax, 149.5, 49, "AO1 / AO2\nmotor phases", W_PHASE, fs=6.8)

    # encoder feedback motor -> ESP32
    wire(ax, [(128, 35.5), (117, 35.5), (117, 46), (112, 46)], W_ENC, arrow=">")
    wire(ax, [(128, 33.5), (115, 33.5), (115, 43.5), (112, 43.5)], W_ENC, arrow=">")
    wlabel(ax, 116.5, 38.8, "A → GPIO2\nB → GPIO1 *", W_ENC, fs=6.6)

    # ---------------- servo ----------------------------------------------
    block(ax, 128, 8, 34, 12, "MG996R servo",
          "steering · 50 Hz PWM\ncenter 90° · ±35°", C_ACT)
    # threaded between the button/LED column (up to y = 26.4 with its padding)
    # and the motor block above it (down to y = 30.6)
    wire(ax, [(110, 42), (110, 27.6), (155, 27.6), (155, 20.5)], W_PWM,
         lw=2.2, arrow=">")
    wlabel(ax, 131, 29.1, "GPIO10 · PWM 50 Hz", W_PWM, fs=6.8)

    # ---------------- sonars (bottom) -------------------------------------
    sonars = [
        ("HC-SR04  FRONT", "TRIG = GPIO16\nECHO = GPIO17 †", 8,  64.0),
        ("HC-SR04  LEFT",  "TRIG = GPIO18\nECHO = GPIO21 †", 39, 74.0),
        ("HC-SR04  RIGHT", "TRIG = GPIO38\nECHO = GPIO39 †", 70, 84.0),
    ]
    for title, sub, bx, drop_x in sonars:
        block(ax, bx, 8, 26, 13, title, sub, C_SENSOR, tfs=8.5, sfs=6.8)
        cx = bx + 13
        wire(ax, [(drop_x, 42), (drop_x, 26), (cx, 26), (cx, 21)],
             W_GPIO, arrow="<>")
    wlabel(ax, 46, 27.8, "TRIG out / ECHO in — one sonar polled per loop()",
           W_GPIO, fs=6.6)

    # ---------------- button + status lights -------------------------------
    block(ax, 100, 19.5, 22, 6.5, "START button", "GPIO42 — arms run",
          C_HMI, tfs=8, sfs=6.5)
    block(ax, 100, 11.5, 22, 6.5, "Status LED", "GPIO36 + 220–470 Ω",
          C_HMI, tfs=8, sfs=6.5)
    block(ax, 100, 3.5, 22, 6.5, "Onboard WS2812", "GPIO48 — on the devkit",
          C_HMI, tfs=8, sfs=6.5)
    wire(ax, [(93, 42), (93, 22.7), (100, 22.7)], W_GPIO, arrow="<")
    wire(ax, [(96, 42), (96, 14.7), (100, 14.7)], W_GPIO, arrow=">")
    wire(ax, [(99, 42), (99, 6.7), (100, 6.7)], W_GPIO, arrow=">")

    # ---------------- what the pixel means ---------------------------------
    kx, ky = 8, 37.5
    ax.text(kx, ky + 2.4, "ONBOARD WS2812 (GPIO48)", fontsize=7.5,
            fontweight="bold", color="#374151")
    for i, (col, name, meaning) in enumerate(PIXEL_STATES):
        yy = ky - i * 2.6
        ax.plot([kx + 0.8], [yy], marker="o", ms=6, color=col, zorder=5)
        ax.text(kx + 2.6, yy, name, fontsize=6.8, fontweight="bold",
                color=col, va="center")
        ax.text(kx + 10, yy, meaning, fontsize=6.6, color="#4b5563",
                va="center")

    # ---------------- legend + power note ---------------------------------
    lx, ly = 8, 79.5
    legend = [("USB", W_USB), ("I²C", W_I2C), ("PWM", W_PWM),
              ("GPIO", W_GPIO), ("encoder", W_ENC), ("motor phase", W_PHASE)]
    ax.text(lx, ly + 2.6, "SIGNALS", fontsize=7.5, fontweight="bold",
            color="#374151")
    for i, (name, col) in enumerate(legend):
        x0 = lx + i * 8.6
        ax.plot([x0, x0 + 3.2], [ly, ly], color=col, lw=2.6)
        ax.text(x0 + 1.6, ly - 2.2, name, ha="center", fontsize=6.4, color=col)

    ax.text(118, 82,
            "POWER (not drawn):  VBAT → TB6612 VM  ·  5 V rail → Pi, servo, sonars  ·  "
            "3V3 → BNO055, OLED, driver VCC",
            fontsize=6.8, color="#b91c1c", ha="left", va="center")

    # ---------------- footnotes -------------------------------------------
    ax.text(8, 1.4,
            "*  encoder A/B intentionally swapped vs silkscreen so forward = positive count (pins.h)      "
            "†  HC-SR04 ECHO is 5 V — divide/level-shift to the 3.3 V GPIO      "
            "4th sonar slot reserved: REAR TRIG=GPIO40 / ECHO=GPIO41  ·  all sonars currently disabled in config.h",
            fontsize=6.6, color="#6b7280", va="center")

    fig.savefig(os.path.join(HERE, "circuit_block_diagram.png"),
                bbox_inches="tight", facecolor=FIG_BG)
    fig.savefig(os.path.join(HERE, "circuit_block_diagram.svg"),
                bbox_inches="tight", facecolor=FIG_BG)
    plt.close(fig)


# ================================================================ figure 2
def pin_table():
    # (device, signal, gpio, note, is_reserved) - reserved rows render grey.
    # the flag lives on the row so inserting one cannot shift a hand written
    # index onto the wrong line
    rows = [
        ("Raspberry Pi 4B ↔ ESP32-S3", "UART1 via USB-to-TTL", "18 / 17",
         "RX / TX · MAVLink 2 @ 115200, /dev/esp32_s3 (mcu_bridge.py)", False),
        ("Debug logs", "UART0 → onboard CH343", "44 / 43",
         "RX / TX · DEBUG_SERIAL, human readable only", False),
        ("BNO055 IMU", "SDA / SCL", "8 / 9", "I²C 0x29 · yaw sampled every 10 ms", False),
        ("OLED SSD1306", "SDA / SCL", "8 / 9 (shared)", "I²C 0x3C · 50 ms refresh", False),
        ("TB6612FNG", "PWMA", "4", "20 kHz, 8-bit duty", False),
        ("", "AIN1 / AIN2", "5 / 6", "direction; both HIGH = brake", False),
        ("", "STBY", "7", "HIGH = driver awake", False),
        ("MG996R servo", "PWM", "10", "50 Hz · center 90° · ±35°", False),
        ("JGA370 encoder", "A / B", "2 / 1",
         "quadrature, 1320 counts/rev · swapped on purpose (pins.h)", False),
        ("Sonar FRONT", "TRIG / ECHO", "16 / 17",
         "HC-SR04 · disabled · ECHO clashes with UART1 TX — unplug it", True),
        ("Sonar LEFT", "TRIG / ECHO", "18 / 21",
         "HC-SR04 · disabled · TRIG clashes with UART1 RX — unplug it", True),
        ("Sonar RIGHT", "TRIG / ECHO", "38 / 39", "HC-SR04 · disabled in config.h", False),
        ("Sonar REAR (slot)", "TRIG / ECHO", "40 / 41", "reserved — no sensor fitted", True),
        ("START button", "input", "42", "arms the run · 3 × 1 s LED countdown", False),
        ("Status LED", "output", "36",
         "lit = ROS 2 bridge connected · plain LED + resistor, not the WS2812", False),
        ("Onboard WS2812", "data", "48",
         "red idle / green linked / flashing blue driving / purple turning", False),
        ("TCS3200 (reserved)", "S0–S3 / OUT", "11–14 / 15", "in pin-map.md, not in firmware", True),
        ("Button 2 / LED 2 (reserved)", "in / out", "47 / 35", "in pin-map.md, not in firmware", True),
    ]

    fig, ax = plt.subplots(figsize=(13, 9), dpi=200)
    fig.patch.set_facecolor(FIG_BG)
    ax.set_xlim(0, 130)
    ax.set_ylim(0, 100)
    ax.axis("off")

    ax.text(65, 96.5, "ESP32-S3 PIN MAP — GORUR GARI 2026", ha="center",
            fontsize=14, fontweight="bold", color="#111827")
    ax.text(65, 92.8, "source: firmware/include/pins.h · firmware/pin-map.md",
            ha="center", fontsize=8, color="#6b7280")

    cols = [(4, "Device"), (40, "Signal"), (62, "GPIO"), (78, "Notes")]
    # rh is what fits len(rows) between `top` and the bottom of the axes: the
    # last row sits at top - rh * len(rows), so adding a row without shrinking
    # this walks it off the figure. 4.7 leaves a little slack at 18 rows.
    top, rh = 88, 4.7
    ax.add_patch(FancyBboxPatch((2, top - 1.6), 126, 4.6,
                                boxstyle="round,pad=0.2",
                                fc="#1f2937", ec="none", zorder=2))
    for cx, name in cols:
        ax.text(cx, top + 0.7, name, fontsize=8.5, fontweight="bold",
                color="white", zorder=3)

    y = top - rh
    for i, (dev, sig, gpio, note, is_reserved) in enumerate(rows):
        if i % 2 == 0:
            ax.add_patch(FancyBboxPatch((2, y - 1.4), 126, rh - 0.6,
                                        boxstyle="round,pad=0.1",
                                        fc="#f3f4f6", ec="none", zorder=1))
        col = "#9ca3af" if is_reserved else "#111827"
        ncol = "#9ca3af" if is_reserved else "#4b5563"
        ax.text(cols[0][0], y + 0.6, dev, fontsize=8, fontweight="bold",
                color=col, zorder=3)
        ax.text(cols[1][0], y + 0.6, sig, fontsize=8, color=col, zorder=3)
        ax.text(cols[2][0], y + 0.6, gpio, fontsize=8, fontweight="bold",
                color="#9ca3af" if is_reserved else "#b45309", zorder=3)
        ax.text(cols[3][0], y + 0.6, note, fontsize=7.3, color=ncol, zorder=3)
        y -= rh

    fig.savefig(os.path.join(HERE, "esp32_pin_map.png"),
                bbox_inches="tight", facecolor=FIG_BG)
    plt.close(fig)


if __name__ == "__main__":
    main_diagram()
    pin_table()
    print("wrote circuit_block_diagram.png/.svg and esp32_pin_map.png")
