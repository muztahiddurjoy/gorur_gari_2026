#!/usr/bin/env python3
"""Generate the circuit block diagram images for gorur_gari_2026.

Wiring is taken from the firmware sources — keep in sync with:
  firmware/include/pins.h      (GPIO assignments)
  firmware/include/config.h    (frequencies, I2C addresses, enables)
  firmware/pin-map.md          (full pin table incl. reserved slots)
  firmware/lib/rgb_status_led/ (what each WS2812 colour means)
  ros2_ws/controls/controls/mcu_bridge.py  (/dev/gorur_mcu MAVLink link)

Usage:  python3 generate_diagrams.py   (needs matplotlib)
        (python3 -s generate_diagrams.py if a NumPy 2 in ~/.local breaks
        the system matplotlib)
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
C_SENSOR  = dict(fc="#dcfce7", ec="#15803d")   # IMU, encoder
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

    # Pi <-> ESP32 (USB cable into the devkit's native "USB" port, MAVLink)
    wire(ax, [(85, 84), (85, 73)], W_USB, lw=2.4, arrow="<>")
    wlabel(ax, 86, 78.5,
           "USB cable → devkit \"USB\" port (native USB CDC, VID 303a)\n"
           "Pi /dev/gorur_mcu  ·  MAVLink 2 only  ·  mcu_bridge.py\n"
           "drive cmds 50 Hz  ·  telemetry every 50 ms",
           W_USB, fs=7, ha="left")

    # ---------------- MCU ----------------------------------------------
    block(ax, 58, 42, 54, 31, "", style=C_MCU)
    ax.text(85, 68.5, "ESP32-S3 DevKitC-1 N16R8", ha="center", fontsize=13,
            fontweight="bold", color="#111827", zorder=4)
    ax.text(85, 65,
            "firmware/ — PlatformIO · Arduino · FreeRTOS\n"
            "core 1: loop() — motor, servo, encoder, status lights, MAVLink,\n"
            "100 ms drive-command watchdog\n"
            "core 0: i2cTask — BOTGyro every pass (416 Hz), OLED @ 200 ms",
            ha="center", va="top", fontsize=7.5, color="#374151", zorder=4)

    # ---------------- I2C bus (left) ------------------------------------
    block(ax, 8, 58, 32, 12, "BOTGyro IMU",
          "ASM330LHH gyro + accel\nI²C addr 0x6A", C_SENSOR)
    block(ax, 8, 42, 32, 12, "OLED SSD1306",
          "128×64 status display\nI²C addr 0x3C", C_HMI)

    trunk_x = 49
    wire(ax, [(58, 56), (trunk_x, 56)], W_I2C, lw=2.2)          # ESP32 -> trunk
    wire(ax, [(trunk_x, 64), (trunk_x, 48)], W_I2C, lw=2.2)     # trunk
    wire(ax, [(trunk_x, 64), (40, 64)], W_I2C, lw=2.2, arrow=">")
    wire(ax, [(trunk_x, 48), (40, 48)], W_I2C, lw=2.2, arrow=">")
    dot(ax, trunk_x, 56, W_I2C)
    dot(ax, 58, 56, W_I2C)
    wlabel(ax, 49, 39.4, "I²C bus (3.3 V, 400 kHz)\nSDA = GPIO8 · SCL = GPIO9", W_I2C, fs=7)

    # ---------------- motor chain (right) --------------------------------
    block(ax, 128, 55, 34, 17, "BTS7960 (IBT-2)",
          "dual half-bridge driver\nB+ = VBAT · 10k pull-downs", C_ACT)
    block(ax, 128, 31, 34, 12, "JGA370 motor",
          "DC gearmotor + quadrature\nencoder (400 cnt/wheel rev)", C_SENSOR)

    ctrl = [("GPIO5 → RPWM · 20 kHz", 69.5, W_PWM),
            ("GPIO6 → LPWM · 20 kHz", 65.0, W_PWM),
            ("GPIO7 → R_EN + L_EN", 60.5, W_GPIO)]
    for label, yy, cc in ctrl:
        wire(ax, [(112, yy), (128, yy)], cc, arrow=">")
        wlabel(ax, 120, yy + 1.5, label, cc, fs=6.4)

    # motor phases driver -> motor
    wire(ax, [(136, 55), (136, 43)], W_PHASE, lw=3.2, arrow=">")
    wire(ax, [(141, 55), (141, 43)], W_PHASE, lw=3.2, arrow=">")
    wlabel(ax, 149.5, 49, "M+ / M−\nmotor phases", W_PHASE, fs=6.8)

    # encoder feedback motor -> ESP32
    wire(ax, [(128, 35.5), (117, 35.5), (117, 46), (112, 46)], W_ENC, arrow=">")
    wire(ax, [(128, 33.5), (115, 33.5), (115, 43.5), (112, 43.5)], W_ENC, arrow=">")
    wlabel(ax, 116.5, 38.8, "C2 → GPIO2\nC1 → GPIO1 *", W_ENC, fs=6.6)

    # ---------------- servo ----------------------------------------------
    block(ax, 128, 8, 34, 12, "MG996R servo",
          "steering · 50 Hz PWM · own 6 V buck\nlimited to STEERING_MIN/MAX_US", C_ACT)
    # threaded between the button/LED column (up to y = 26.4 with its padding)
    # and the motor block above it (down to y = 30.6)
    wire(ax, [(110, 42), (110, 27.6), (155, 27.6), (155, 20.5)], W_PWM,
         lw=2.2, arrow=">")
    wlabel(ax, 131, 29.1, "GPIO16 · PWM 50 Hz", W_PWM, fs=6.8)

    # ---------------- button + status lights -------------------------------
    block(ax, 100, 19.5, 22, 6.5, "START = BOOT button", "GPIO0 — on the devkit",
          C_HMI, tfs=8, sfs=6.5)
    block(ax, 100, 11.5, 22, 6.5, "Status LED", "GPIO4 + 220–470 Ω",
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
            "POWER (not drawn):  VBAT → BTS7960 B+  ·  6 V buck → servo  ·  "
            "5 V rail → Pi, ESP32  ·  3V3 → BOTGyro, OLED",
            fontsize=6.8, color="#b91c1c", ha="left", va="center")

    # ---------------- footnotes -------------------------------------------
    ax.text(8, 1.4,
            "*  encoder phases ordered so forward = positive count (pins.h)      "
            "GPIO35–37 belong to the N16R8's octal PSRAM and are never used      "
            "GPIO40/41 unused (old UART link)      no sonars on this revision",
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
        ("Raspberry Pi 4B ↔ ESP32-S3", "native USB D− / D+", "19 / 20",
         "devkit \"USB\" port · USB CDC · MAVLink only · Pi /dev/gorur_mcu", False),
        ("BOTGyro IMU", "SDA / SCL", "8 / 9", "I²C 0x6A (0x6B if SA0 high) · 400 kHz", False),
        ("OLED SSD1306", "SDA / SCL", "8 / 9 (shared)", "I²C 0x3C · 200 ms refresh", False),
        ("BTS7960 (IBT-2)", "RPWM", "5", "forward · 20 kHz, 8-bit · 10k pull-down", False),
        ("", "LPWM", "6", "reverse · 20 kHz, 8-bit · 10k pull-down", False),
        ("", "R_EN + L_EN", "7", "tied · HIGH = bridge on · 10k pull-down", False),
        ("MG996R servo", "PWM", "16", "50 Hz · STEERING_MIN/CENTER/MAX_US · own 6 V buck", False),
        ("JGA370 encoder", "C1 / C2", "1 / 2", "quadrature, 3.3 V · forward counts up (pins.h)", False),
        ("START button", "BOOT button", "0",
         "strapping pin · INPUT_PULLUP, read after boot · 30 ms debounce", False),
        ("Status LED", "output", "4",
         "link level · start countdown · IMU calibrating / fault patterns", False),
        ("Onboard WS2812", "data", "48",
         "red idle / green linked / flashing blue driving / purple turning", False),
        ("Old UART link", "—", "40 / 41", "unused, deliberately left free", True),
        ("Octal PSRAM (N16R8)", "—", "35–37", "owned by the module, never used", True),
    ]

    fig, ax = plt.subplots(figsize=(13, 9), dpi=200)
    fig.patch.set_facecolor(FIG_BG)
    ax.set_xlim(0, 130)
    ax.set_ylim(0, 100)
    ax.axis("off")

    ax.text(65, 96.5, "ESP32-S3 PIN MAP — GORUR GARI 2026", ha="center",
            fontsize=14, fontweight="bold", color="#111827")
    ax.text(65, 92.8, "ESP32-S3-DevKitC-1 N16R8 · source: firmware/include/pins.h · firmware/pin-map.md",
            ha="center", fontsize=8, color="#6b7280")

    cols = [(4, "Device"), (40, "Signal"), (62, "GPIO"), (78, "Notes")]
    top, rh = 88, 5.0
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
