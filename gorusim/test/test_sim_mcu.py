"""The firmware behaviour sim_mcu reproduces, checked against the firmware's sources."""
import math
import os
import re

import pytest
from gorusim.sim_mcu import (ENCODER_COUNTS_PER_REV, SENSOR_TX_INTERVAL_S,
                             SERVO_CORRECTION_DEG, START_BLINK_COUNT,
                             START_BLINK_INTERVAL_S, StatusLedModel,
                             duty_after_stiction, duty_from_throttle,
                             heading_from_yaw, horn_from_wire)

FIRMWARE = os.path.join(os.path.dirname(__file__), '..', '..', 'firmware')


def firmware_file(*parts):
    with open(os.path.join(FIRMWARE, *parts)) as handle:
        return handle.read()


def test_constants_match_the_firmware():
    config = firmware_file('include', 'config.h')
    assert re.search(r'ENCODER_COUNTS_PER_REV = (\d+)', config).group(1) == str(
        ENCODER_COUNTS_PER_REV)
    assert re.search(r'SENSOR_TX_INTERVAL_MS = (\d+)', config).group(1) == str(
        int(SENSOR_TX_INTERVAL_S * 1000))
    steering = firmware_file('lib', 'steering_control', 'steering_control.h')
    assert re.search(r'servoCorrection = (\d+)', steering).group(1) == str(SERVO_CORRECTION_DEG)


def test_full_scale_int8_is_under_half_duty():
    assert duty_from_throttle(127) == pytest.approx(127 / 255)
    assert duty_from_throttle(-128) == pytest.approx(-128 / 255)
    assert duty_from_throttle(0) == 0.0


def test_stopped_axle_ignores_a_weak_command():
    assert duty_after_stiction(0.10, 0.0, 0.15) == 0.0
    assert duty_after_stiction(-0.10, 0.0, 0.15) == 0.0
    assert duty_after_stiction(0.20, 0.0, 0.15) == 0.20


def test_rolling_axle_keeps_a_weak_command():
    assert duty_after_stiction(0.10, 5.0, 0.15) == 0.10


def test_wire_centre_is_horn_centre():
    assert horn_from_wire(90) == 0


def test_correction_makes_travel_uneven():
    assert horn_from_wire(180) == 90
    assert horn_from_wire(23) == -67
    # Below wire 23 the servo library is already pinned at 0.
    assert horn_from_wire(0) == -67


def test_wire_is_clamped_before_the_correction():
    assert horn_from_wire(250) == 90
    assert horn_from_wire(-5) == -67


def test_heading_grows_clockwise_from_power_on():
    zero = 0.7
    assert heading_from_yaw(zero, zero) == pytest.approx(0.0)
    # A left turn (counter-clockwise, +yaw) reads as a small step back from 360.
    assert heading_from_yaw(zero + math.radians(10.0), zero) == pytest.approx(350.0)
    assert heading_from_yaw(zero - math.radians(10.0), zero) == pytest.approx(10.0)


def test_heading_stays_in_range_for_a_vanishing_negative_angle():
    assert 0.0 <= heading_from_yaw(1e-18, 0.0) < 360.0


# ---- StatusLed (firmware/lib/status_led) ------------------------------------


def run_led(led, start, stop, step=0.001):
    """Call update() like loop() does; return the lit/dark level at each step."""
    levels = []
    t = start
    while t < stop:
        led.update(t)
        levels.append(led.state)
        t += step
    return levels


def test_start_blink_matches_the_firmware():
    config = firmware_file('include', 'config.h')
    assert int(re.search(r'START_BLINK_COUNT = (\d+)', config).group(1)) == START_BLINK_COUNT
    assert int(re.search(r'START_BLINK_INTERVAL_MS = (\d+)', config).group(1)) == int(
        START_BLINK_INTERVAL_S * 1000)


def test_led_starts_dark_and_follows_set():
    led = StatusLedModel()
    assert not led.state
    led.set(True)
    assert led.state
    led.set(False)
    assert not led.state


def test_blink_runs_its_cycles_then_lands_back():
    led = StatusLedModel()
    led.set(True)
    led.blink(3, 1.0, now=0.0)
    levels = run_led(led, 0.0, 3.5)
    # lit -> dark is one edge per cycle: three falling edges, no more
    falls = sum(1 for a, b in zip(levels, levels[1:]) if a and not b)
    assert falls == 3
    assert not led.is_blinking()
    assert led.state  # back to the level before the blink


def test_set_during_a_blink_only_moves_the_landing_level():
    led = StatusLedModel()
    led.set(True)
    led.blink(2, 1.0, now=0.0)
    led.update(0.6)
    assert not led.state     # dark half of the first cycle
    led.set(True)            # does not cut the blink short
    assert not led.state
    led.set(False)
    run_led(led, 0.6, 2.5)
    assert not led.is_blinking()
    assert not led.state


def test_restarting_a_blink_keeps_the_original_landing_level():
    led = StatusLedModel()
    led.set(False)
    led.blink(3, 1.0, now=0.0)
    led.blink(3, 1.0, now=0.2)  # lit at this moment, but that is the blink's own output
    run_led(led, 0.2, 4.0)
    assert not led.state


def test_zero_blink_is_ignored():
    led = StatusLedModel()
    led.blink(0, 1.0, now=0.0)
    led.blink(3, 0.0, now=0.0)
    assert not led.is_blinking()
    assert not led.state
