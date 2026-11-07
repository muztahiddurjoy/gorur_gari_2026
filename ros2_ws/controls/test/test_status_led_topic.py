"""
The status LED topics on mcu_bridge, and the firmware behaviour they lean on.

There is no LED message on the wire: /status_led and /status_led/blink are sent
as the connect frame, which the firmware turns into status_led.set(connected).
If someone ever changes what that frame does in main.cpp, these topics stop
doing what they say, so the last test reads main.cpp and checks.

    python3 -m pytest ros2_ws/controls/test/test_status_led_topic.py -v
"""

import math
import os
import re

import pytest

from controls.mcu_bridge import STATUS_LED_MAX_BLINK_HZ, blink_half_period

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
FIRMWARE_MAIN = os.path.join(REPO_ROOT, 'firmware', 'src', 'main.cpp')


def test_one_blink_is_a_full_on_off_cycle():
    # 1 Hz: lit for half a second, dark for half a second
    assert blink_half_period(1.0) == pytest.approx(0.5)
    assert blink_half_period(2.0) == pytest.approx(0.25)


@pytest.mark.parametrize('rate', [0.0, -1.0, math.nan, -math.inf])
def test_zero_negative_or_garbage_stops_the_blink(rate):
    assert blink_half_period(rate) is None


def test_fast_requests_are_capped_not_refused():
    assert blink_half_period(1000.0) == pytest.approx(0.5 / STATUS_LED_MAX_BLINK_HZ)
    assert blink_half_period(math.inf) == pytest.approx(0.5 / STATUS_LED_MAX_BLINK_HZ)
    assert blink_half_period(50.0, max_rate_hz=5.0) == pytest.approx(0.1)


def test_firmware_drives_the_led_from_the_connect_frame():
    if not os.path.isfile(FIRMWARE_MAIN):
        pytest.skip(f'firmware tree not next to this package: {FIRMWARE_MAIN}')
    with open(FIRMWARE_MAIN, encoding='utf-8') as handle:
        source = handle.read()
    handler = re.search(
        r'MAVLINK_MSG_ID_gorur_gari_ros2_to_mcu_connect_msg\)\{(.*?)\n\s*\}', source, re.S)
    assert handler, 'connect frame handler not found in main.cpp'
    assert re.search(r'status_led\.set\(\s*connected\s*!=\s*0\s*\)', handler.group(1))
