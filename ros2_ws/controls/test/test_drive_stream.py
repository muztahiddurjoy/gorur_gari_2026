"""
The drive command stream mcu_bridge feeds the firmware's 100 ms watchdog.

The firmware (firmware/include/config.h, CMD_TIMEOUT_MS) brakes the car as
soon as no valid drive command has arrived for 100 ms. mcu_bridge therefore
re-sends the latest /cmd_vel at 50 Hz while it is fresh and goes quiet once it
is stale. These tests check the constants on both ends agree, run the real
node against a pseudo terminal to measure what actually goes out on the wire,
and check the firmware's STATUSTEXT reaches the node's log decoder.

    python3 -m pytest ros2_ws/controls/test/test_drive_stream.py -v
"""

import os
import re
import shutil
import subprocess
import tempfile
import time

import pytest

from controls import mcu_to_ros2, ros2_to_mcu
from controls.mcu_bridge import (
    CMD_VEL_STALE_S, DRIVE_TX_INTERVAL_S, FIRMWARE_CMD_TIMEOUT_S, MCU_FALLBACK_PORT, MCU_PORT,
    MCU_RECONNECT_INTERVAL_S, STATUSTEXT_MSG_ID, decode_mcu_statustext, drive_command_is_fresh,
    open_mcu_port)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
FIRMWARE_CONFIG = os.path.join(REPO_ROOT, 'firmware', 'include', 'config.h')
FIRMWARE_INCLUDE = os.path.join(REPO_ROOT, 'firmware', 'include')
FIRMWARE_MAV_MSGS = os.path.join(REPO_ROOT, 'firmware', 'lib', 'mav_msgs')
BOT_CONFIG = os.path.join(REPO_ROOT, 'ros2_ws', 'config', 'bot_config.yaml')
UDEV_RULES = os.path.join(REPO_ROOT, 'ros2_ws', 'config', '99-gorur.rules')


def read_text(path):
    if not os.path.isfile(path):
        pytest.skip(f'not next to this package: {path}')
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def firmware_const(name):
    match = re.search(rf'\b{name}\s*=\s*(\d+)', read_text(FIRMWARE_CONFIG))
    assert match, f'{name} not found in {FIRMWARE_CONFIG}'
    return int(match.group(1))


# ══════════════════════════════════════════════════════════════════════
# Constants on both ends of the wire
# ══════════════════════════════════════════════════════════════════════

def test_firmware_timeout_matches_the_bridge():
    assert firmware_const('CMD_TIMEOUT_MS') == round(FIRMWARE_CMD_TIMEOUT_S * 1000)


def test_50hz_rides_through_four_lost_frames():
    # four lost frames in a row leave a 5-interval gap between the two that
    # arrive; the firmware must still be inside its timeout at that point
    timeout_ms = firmware_const('CMD_TIMEOUT_MS')
    assert DRIVE_TX_INTERVAL_S == pytest.approx(0.02)
    assert 5 * DRIVE_TX_INTERVAL_S * 1000 <= timeout_ms


def test_baud_matches():
    from controls.mcu_bridge import MCU_BAUDRATE
    assert firmware_const('MAV_SERIAL_BAUD') == MCU_BAUDRATE


def bridge_param(name):
    match = re.search(r'^mcu_bridge:\s*\n\s*ros__parameters:(.*?)^\S',
                      read_text(BOT_CONFIG), re.S | re.M)
    assert match, 'mcu_bridge section not found in bot_config.yaml'
    value = re.search(rf'^\s*{name}:\s*(\S+)', match.group(1), re.M)
    assert value, f'{name} not set in the mcu_bridge section'
    return value.group(1)


def test_port_is_the_stable_usb_name_everywhere():
    assert MCU_PORT == '/dev/gorur_mcu'
    assert MCU_FALLBACK_PORT == '/dev/ttyACM0'
    assert bridge_param('port') == '/dev/gorur_mcu'
    assert bridge_param('fallback_port') == '/dev/ttyACM0'


def test_udev_rules_name_the_mcu_and_the_lidar():
    rules = [line for line in read_text(UDEV_RULES).splitlines()
             if line.strip() and not line.lstrip().startswith('#')]
    mcu = [r for r in rules if 'SYMLINK+="gorur_mcu"' in r]
    lidar = [r for r in rules if 'SYMLINK+="rplidar"' in r]
    assert len(mcu) == 1 and 'ATTRS{idVendor}=="303a"' in mcu[0]
    assert len(lidar) == 1
    assert 'ATTRS{idVendor}=="10c4"' in lidar[0] and 'ATTRS{idProduct}=="ea60"' in lidar[0]
    for rule in rules:
        assert 'SUBSYSTEM=="tty"' in rule and 'MODE="0666"' in rule


class RecordingSerial:
    """Stands in for serial.Serial and records the order things happen in."""

    events = []

    def __init__(self):
        object.__setattr__(self, 'is_open', False)

    def __setattr__(self, name, value):
        RecordingSerial.events.append((name, value, self.is_open))
        object.__setattr__(self, name, value)

    def open(self):
        RecordingSerial.events.append(('open', None, False))
        object.__setattr__(self, 'is_open', True)


def test_open_mcu_port_cannot_reset_the_esp32():
    RecordingSerial.events = []
    port = open_mcu_port('/dev/gorur_mcu', 115200, serial_class=RecordingSerial)
    names = [e[0] for e in RecordingSerial.events]
    settings = {e[0]: e[1] for e in RecordingSerial.events}
    # no hardware flow control to drive the modem lines on its own
    assert settings['dsrdtr'] is False and settings['rtscts'] is False
    assert settings['timeout'] == 0
    # both lines dropped after opening, RTS before DTR: the other order passes
    # through RTS high / DTR low, which resets the ESP32-S3
    opened = names.index('open')
    rts = max(i for i, n in enumerate(names) if n == 'rts')
    dtr = max(i for i, n in enumerate(names) if n == 'dtr')
    assert opened < rts < dtr
    assert port.rts is False and port.dtr is False


# ══════════════════════════════════════════════════════════════════════
# Freshness
# ══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('age', [0.0, 0.01, 0.1, CMD_VEL_STALE_S - 1e-3])
def test_fresh(age):
    assert drive_command_is_fresh(age)


@pytest.mark.parametrize('age', [None, CMD_VEL_STALE_S, 0.5, 10.0, -0.1])
def test_stale_or_nothing_yet(age):
    assert not drive_command_is_fresh(age)


# ══════════════════════════════════════════════════════════════════════
# STATUSTEXT from the firmware
# ══════════════════════════════════════════════════════════════════════

def parse_with_bridge_dialect(data):
    """Run bytes through the same dialect parser mcu_bridge uses."""
    parser = mcu_to_ros2.MAVLink(None)
    parser.robust_parsing = True
    return parser.parse_buffer(data) or []


def test_statustext_from_pymavlink_reaches_the_decoder():
    from pymavlink.dialects.v20 import common
    tx = common.MAVLink(None, srcSystem=1, srcComponent=200)
    frame = tx.statustext_encode(3, b'IMU fault: gyro stopped answering, retrying').pack(tx)
    msgs = parse_with_bridge_dialect(frame)
    assert [m.get_type() for m in msgs] == [f'UNKNOWN_{STATUSTEXT_MSG_ID}']
    assert decode_mcu_statustext(msgs[0].get_msgbuf()) == (
        3, 'IMU fault: gyro stopped answering, retrying')


def test_corrupted_statustext_is_not_decoded():
    from pymavlink.dialects.v20 import common
    tx = common.MAVLink(None, srcSystem=1, srcComponent=200)
    frame = bytearray(tx.statustext_encode(4, b'hello').pack(tx))
    frame[12] ^= 0x20  # a payload bit; the CRC no longer matches
    assert decode_mcu_statustext(frame) is None
    assert decode_mcu_statustext(b'') is None


def test_statustext_packed_by_the_firmware_header():
    """Compile firmware/include/mavlink_link.h on the host and decode its bytes."""
    cxx = shutil.which('g++')
    if cxx is None:
        pytest.skip('no g++ on this machine')
    read_text(os.path.join(FIRMWARE_INCLUDE, 'mavlink_link.h'))
    source = r'''
        #include <cstdio>
        #include "mavlink_link.h"
        int main() {
            mavlink_message_t msg; uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_pack_statustext(1, 200, &msg, STATUSTEXT_SEVERITY_WARNING,
                "I2C unreliable at 400 kHz, bus now 100 kHz");
            fwrite(buf, 1, mavlink_msg_to_send_buffer(buf, &msg), stdout);
            mavlink_msg_gorur_gari_mcu_to_ros2_msg_pack(1, 200, &msg,
                1234, 10, 1, 90, -42.5f, 255, 255, 255, 255, 0);
            fwrite(buf, 1, mavlink_msg_to_send_buffer(buf, &msg), stdout);
            return 0;
        }
    '''
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'pack.cpp')
        exe = os.path.join(tmp, 'pack')
        with open(src, 'w', encoding='utf-8') as handle:
            handle.write(source)
        subprocess.run([cxx, '-std=c++17', '-I', FIRMWARE_INCLUDE, '-I', FIRMWARE_MAV_MSGS,
                        src, '-o', exe], check=True)
        data = subprocess.run([exe], check=True, capture_output=True).stdout

    msgs = parse_with_bridge_dialect(data)
    assert [m.get_type() for m in msgs] == [
        f'UNKNOWN_{STATUSTEXT_MSG_ID}', 'GORUR_GARI_MCU_TO_ROS2_MSG']
    assert decode_mcu_statustext(msgs[0].get_msgbuf()) == (
        4, 'I2C unreliable at 400 kHz, bus now 100 kHz')
    # and the telemetry right behind it still parses
    assert msgs[1].heading == pytest.approx(-42.5)
    assert msgs[1].encoder_count == 1234


# ══════════════════════════════════════════════════════════════════════
# The real node, over a pseudo terminal
# ══════════════════════════════════════════════════════════════════════

def test_node_streams_at_50hz_while_fresh_and_stops_when_stale():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor

    from controls.mcu_bridge import MCUBridgeNode

    master, slave = os.openpty()
    os.set_blocking(master, False)
    rclpy.init(args=['--ros-args', '-p', f'port:={os.ttyname(slave)}',
                     '-p', "fallback_port:=''"])
    try:
        bridge = MCUBridgeNode()
        pub_node = rclpy.create_node('drive_stream_test_publisher')
        pub = pub_node.create_publisher(Twist, '/cmd_vel', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(pub_node)

        stamps = []   # arrival time of each drive frame
        frames = []

        def pump(seconds, publish_every=None):
            end = time.monotonic() + seconds
            next_pub = time.monotonic()
            parser = ros2_to_mcu.MAVLink(None)
            parser.robust_parsing = True
            while time.monotonic() < end:
                if publish_every is not None and time.monotonic() >= next_pub:
                    msg = Twist()
                    msg.linear.x = 0.3
                    msg.angular.z = -0.5
                    pub.publish(msg)
                    next_pub += publish_every
                executor.spin_once(timeout_sec=0.002)
                try:
                    chunk = os.read(master, 4096)
                except BlockingIOError:
                    chunk = b''
                for m in (parser.parse_buffer(chunk) or []) if chunk else []:
                    if m.get_type() == 'GORUR_GARI_ROS2_TO_MCU_MSG':
                        stamps.append(time.monotonic())
                        frames.append((m.throttle, m.steering))

        # /cmd_vel at only 5 Hz for 1 s: the node must still stream at 50 Hz
        pump(1.0, publish_every=0.2)
        streamed = len(frames)
        last_cmd = time.monotonic()
        # then nothing: within CMD_VEL_STALE_S (+ one tick) the stream stops
        pump(1.0)

        assert streamed >= 40, f'only {streamed} drive frames in 1 s at 50 Hz'
        gaps = [b - a for a, b in zip(stamps, stamps[1:]) if a < last_cmd]
        # the firmware tolerates 100 ms; the stream must never come close
        assert max(gaps) < 0.06, f'largest gap between drive frames {max(gaps):.3f} s'
        # what handle_cmd_vel works out from the node's own parameters (the
        # in-code defaults here, no bot_config.yaml): throttle with and
        # without the launch boost, steering = centre + angular.z * gain
        throttle = int(0.3 * bridge.throttle_scale)
        boosted = int(0.3 * bridge.launch_boost_gain * bridge.throttle_scale)
        steering = int(bridge.steering_center_deg - 0.5 * bridge.steering_gain_deg)
        assert set(frames) <= {(throttle, steering), (boosted, steering)}
        late = [t for t in stamps if t > last_cmd + CMD_VEL_STALE_S + 0.05]
        assert not late, f'{len(late)} drive frames sent after /cmd_vel went stale'

        executor.remove_node(bridge)
        executor.remove_node(pub_node)
        bridge.destroy_node()
        pub_node.destroy_node()
    finally:
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


def test_node_reconnects_when_the_port_comes_back():
    """
    Survive the port vanishing and coming back.

    The port goes away (ESP32 reset, cable pulled) and returns: no crash, a
    retry every MCU_RECONNECT_INTERVAL_S, and the drive stream picks up again
    on the new port, announced with a fresh connect notice.
    """
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor

    from controls.mcu_bridge import MCUBridgeNode

    tmp = tempfile.mkdtemp()
    link = os.path.join(tmp, 'gorur_mcu')   # like the udev symlink
    master, slave = os.openpty()
    os.set_blocking(master, False)
    os.symlink(os.ttyname(slave), link)
    rclpy.init(args=['--ros-args', '-p', f'port:={link}', '-p', "fallback_port:=''"])
    try:
        bridge = MCUBridgeNode()
        pub_node = rclpy.create_node('drive_reconnect_test_publisher')
        pub = pub_node.create_publisher(Twist, '/cmd_vel', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(pub_node)
        twist = Twist()
        twist.linear.x = 0.2

        def pump(seconds, fd):
            types = []
            parser = ros2_to_mcu.MAVLink(None)
            parser.robust_parsing = True
            end = time.monotonic() + seconds
            next_pub = time.monotonic()
            while time.monotonic() < end:
                if time.monotonic() >= next_pub:
                    pub.publish(twist)
                    next_pub += 0.05
                executor.spin_once(timeout_sec=0.002)
                if fd is None:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except (BlockingIOError, OSError):
                    chunk = b''
                types += [m.get_type() for m in (parser.parse_buffer(chunk) or [])]
            return types

        before = pump(1.5, master)
        assert before.count('GORUR_GARI_ROS2_TO_MCU_MSG') > 40
        assert 'GORUR_GARI_ROS2_TO_MCU_CONNECT_MSG' in before

        # the port goes away: the other end closes and the device node vanishes
        os.close(master)
        os.close(slave)
        os.remove(link)
        pump(0.3, None)
        assert not bridge.mcu_connected, 'the bridge did not notice the port going away'

        # stays away for a while: retried, never crashes
        pump(1.5, None)
        assert not bridge.mcu_connected

        # and comes back as a new tty under the same name
        master, slave = os.openpty()
        os.set_blocking(master, False)
        os.symlink(os.ttyname(slave), link)
        after = pump(MCU_RECONNECT_INTERVAL_S + 1.5, master)
        assert bridge.mcu_connected
        assert after.count('GORUR_GARI_ROS2_TO_MCU_MSG') > 20
        # announced again, so the MCU's status LED comes back on
        assert 'GORUR_GARI_ROS2_TO_MCU_CONNECT_MSG' in after

        executor.remove_node(bridge)
        executor.remove_node(pub_node)
        bridge.destroy_node()
        pub_node.destroy_node()
    finally:
        rclpy.shutdown()
        for fd in (master, slave):
            try:
                os.close(fd)
            except OSError:
                pass
        shutil.rmtree(tmp, ignore_errors=True)
