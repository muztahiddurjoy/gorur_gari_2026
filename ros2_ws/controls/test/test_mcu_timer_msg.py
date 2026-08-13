"""
The run timer message, checked across all three copies of its definition.

mav_msg/ros2_to_mcu.xml is the source of truth, but it is compiled twice: into
the pymavlink dialect this package imports, and into the C headers the
firmware includes. Nothing at runtime notices if only one of those was
regenerated - the frames simply stop decoding, on a car, at a competition. So
the tests below read the C headers off disk and check them against the Python
dialect, and check the wire constants in mcu_bridge and the firmware agree too.

    python3 -m pytest ros2_ws/controls/test/test_mcu_timer_msg.py -v
"""

import importlib.util
import os
import re
import struct

import pytest

from controls import ros2_to_mcu
from controls.mcu_bridge import RUN_TIMER_ELAPSED_MS_MAX, run_time_to_wire_ms

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
DIALECT_HEADER = os.path.join(
    REPO_ROOT, 'firmware', 'lib', 'mav_msgs', 'ros2_to_mcu', 'ros2_to_mcu.h')
RUN_TIMER_HEADER = os.path.join(
    REPO_ROOT, 'firmware', 'lib', 'run_timer', 'run_timer.h')
RUN_TIMER_NODE = os.path.join(
    REPO_ROOT, 'ros2_ws', 'autonomy', 'autonomy', 'run_timer.py')

TIMER_MSG_ID = 50004


def read_firmware(path):
    if not os.path.isfile(path):
        pytest.skip(f'firmware tree not next to this package: {path}')
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def load_run_timer_node():
    """
    Import autonomy's run_timer off disk, without depending on the package.

    controls does not build against autonomy and should not have to, but the
    two format the same number and the whole point below is that they agree.
    """
    if not os.path.isfile(RUN_TIMER_NODE):
        pytest.skip(f'autonomy package not next to this one: {RUN_TIMER_NODE}')
    spec = importlib.util.spec_from_file_location('_run_timer', RUN_TIMER_NODE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def firmware_message_crcs():
    """{msgid: crc_extra} out of the C dialect's MAVLINK_MESSAGE_CRCS table."""
    table = re.search(r'#define MAVLINK_MESSAGE_CRCS \{(.*)\}\n',
                      read_firmware(DIALECT_HEADER))
    assert table, 'MAVLINK_MESSAGE_CRCS not found in ' + DIALECT_HEADER
    return {int(msgid): int(crc)
            for msgid, crc in re.findall(r'\{(\d+),\s*(\d+),', table.group(1))}


# ══════════════════════════════════════════════════════════════════════
# The Python dialect
# ══════════════════════════════════════════════════════════════════════

def test_timer_message_is_in_the_dialect():
    assert ros2_to_mcu.MAVLINK_MSG_ID_GORUR_GARI_ROS2_TO_MCU_TIMER_MSG == TIMER_MSG_ID
    assert TIMER_MSG_ID in ros2_to_mcu.mavlink_map


def test_timer_message_round_trips():
    """Everything the OLED needs has to survive the wire unchanged."""
    sender = ros2_to_mcu.MAVLink(None, srcSystem=2, srcComponent=1)
    receiver = ros2_to_mcu.MAVLink(None)

    # a fresh clock, a tick, a plausible run, and the ceiling of both fields
    for elapsed_ms, state in [(0, 0), (100, 1), (194_300, 1), (0xFFFFFFFF, 2)]:
        frame = ros2_to_mcu.MAVLink_gorur_gari_ros2_to_mcu_timer_msg_message(
            elapsed_ms=elapsed_ms, state=state).pack(sender)

        decoded = receiver.decode(bytearray(frame))
        assert decoded.get_msgId() == TIMER_MSG_ID
        assert decoded.elapsed_ms == elapsed_ms
        assert decoded.state == state


def test_timer_message_payload_is_five_bytes():
    """
    The payload is uint32 then uint8, little endian, no padding.

    That is the layout the firmware's generated accessors read back, and five
    bytes at 10 Hz is nothing next to the telemetry already on the link.
    """
    cls = ros2_to_mcu.mavlink_map[TIMER_MSG_ID]
    assert cls.unpacker.format in ('<IB', b'<IB')
    assert cls.unpacker.size == 5
    assert cls.fieldnames == ['elapsed_ms', 'state']


def test_the_existing_messages_still_decode():
    """
    Adding a message must not disturb the two that were already flying.

    Their ids and crc_extras are the whole compatibility contract with a
    firmware that has not been reflashed yet.
    """
    assert ros2_to_mcu.MAVLINK_MSG_ID_GORUR_GARI_ROS2_TO_MCU_MSG == 50002
    assert ros2_to_mcu.MAVLINK_MSG_ID_GORUR_GARI_ROS2_TO_MCU_CONNECT_MSG == 50003
    assert ros2_to_mcu.mavlink_map[50002].crc_extra == 194
    assert ros2_to_mcu.mavlink_map[50003].crc_extra == 149


# ══════════════════════════════════════════════════════════════════════
# ...against the C headers the firmware compiles
# ══════════════════════════════════════════════════════════════════════

def test_firmware_and_python_dialects_agree():
    """Same message ids, same crc_extras, or nothing decodes on either end."""
    firmware = firmware_message_crcs()
    python = {msgid: cls.crc_extra for msgid, cls in ros2_to_mcu.mavlink_map.items()}
    assert firmware == python, (
        'the C headers and the pymavlink dialect were generated from '
        'different versions of mav_msg/ros2_to_mcu.xml - regenerate both')


def test_firmware_declares_the_timer_message():
    header = read_firmware(DIALECT_HEADER)
    assert 'mavlink_msg_gorur_gari_ros2_to_mcu_timer_msg.h' in header
    assert firmware_message_crcs()[TIMER_MSG_ID] == \
        ros2_to_mcu.mavlink_map[TIMER_MSG_ID].crc_extra


@pytest.mark.parametrize('seconds,expected_ms', [
    (0.0, 0),
    (0.0999, 99),
    (1.5, 1500),
    (194.3, 194_300),
    (-1.0, 0),                          # a clock cannot read backwards
    # nonsense zeroes the clock, exactly as run_timer's own formatter does,
    # so the panel shows 00:00.0 rather than a number nobody can explain
    (float('nan'), 0),
    (float('inf'), 0),
    (float('-inf'), 0),
    (1e12, RUN_TIMER_ELAPSED_MS_MAX),   # merely huge is clamped, never wrapped
])
def test_run_time_to_wire_ms(seconds, expected_ms):
    assert run_time_to_wire_ms(seconds) == expected_ms


def test_the_screen_and_the_logs_never_show_different_run_times():
    """
    The OLED and /run_time_str have to agree, tenth for tenth.

    They are computed twice from two different numbers: run_timer formats the
    float64 it holds, while the panel formats the milliseconds that survived
    a float32 topic and an integer truncation. Both paths are swept here
    across a five minute run, one millisecond at a time.
    """
    format_run_time = load_run_timer_node().format_run_time

    def firmware_line(elapsed_ms):
        """Format a millisecond count the way lib/run_timer does."""
        tenths = elapsed_ms // 100
        return f'{tenths // 600:02d}:{(tenths // 10) % 60:02d}.{tenths % 10}'

    for ms in range(0, 300_000):
        seconds = ms / 1000.0
        # std_msgs/Float32 is what actually crosses between the two nodes
        as_float32 = struct.unpack('<f', struct.pack('<f', seconds))[0]
        assert firmware_line(run_time_to_wire_ms(as_float32)) == \
            format_run_time(as_float32), f'the two disagree at {ms} ms'


def test_wire_state_values_match_the_firmware_enum():
    """
    Two hand written tables, one meaning.

    mcu_bridge encodes the stopwatch state as a number and lib/run_timer
    decodes it, and neither is generated from the other.
    """
    from controls.mcu_bridge import RUN_TIMER_WIRE_STATE

    header = read_firmware(RUN_TIMER_HEADER)
    firmware_states = dict(re.findall(r'RUN_TIMER_(\w+)\s*=\s*(\d+)', header))
    assert firmware_states, 'no RunTimerState enum found in ' + RUN_TIMER_HEADER

    for name, value in RUN_TIMER_WIRE_STATE.items():
        assert name in firmware_states, f'the firmware has no RUN_TIMER_{name}'
        assert int(firmware_states[name]) == value, (
            f'{name} is {value} in mcu_bridge but '
            f'{firmware_states[name]} in the firmware')
