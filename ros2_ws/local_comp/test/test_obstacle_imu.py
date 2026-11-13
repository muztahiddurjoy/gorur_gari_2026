"""Unit tests for IMU heading fusion in ObstacleRound."""

import math
import time

import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from local_comp.obstacle_round import ObstacleRound


@pytest.fixture(autouse=True)
def ros_context():
    """Initialize and shut down rclpy cleanly for each test."""
    rclpy.init()
    yield
    rclpy.try_shutdown()


def make_node():
    """Create a fresh ObstacleRound node in RUNNING state with zeroed IMU."""
    node = ObstacleRound()
    node.button_callback(Bool(data=True))
    node.start_run()
    return node


def make_scan(range_fn):
    """Build a -90..+90 deg 1-deg LaserScan from range_fn(angle_rad)."""
    angles = [math.radians(deg) for deg in range(-90, 91)]
    msg = LaserScan()
    msg.angle_min = -math.pi / 2.0
    msg.angle_max = math.pi / 2.0
    msg.angle_increment = math.radians(1.0)
    msg.range_min = 0.05
    msg.range_max = 12.0
    msg.ranges = [float(range_fn(a)) for a in angles]
    return msg


def test_heading_zeroed_on_start_run():
    """Verify idle MCU drift does not accumulate and start_run resets state."""
    node = ObstacleRound()
    try:
        node.heading_callback(Float32(data=120.0))
        node.heading_callback(Float32(data=135.0))
        assert node.yaw_deg == pytest.approx(0.0)

        node.button_callback(Bool(data=True))
        assert node.state == 'COUNTDOWN'
        node.heading_callback(Float32(data=145.0))
        assert node.yaw_deg == pytest.approx(0.0)

        node.start_run()
        assert node.state == 'RUNNING'
        assert node.yaw_deg == pytest.approx(0.0)
        assert node.yaw_rate_dps == pytest.approx(0.0)
    finally:
        node.destroy_node()


def test_heading_wrapping_and_clockwise_sign():
    """Verify 0/360 and +-180 wraparound and CW-positive to CCW-positive flip."""
    node = make_node()
    try:
        node.heading_clockwise = True
        # Cross 359 -> 1 (+2 deg CW == -2 deg CCW)
        node.heading_callback(Float32(data=359.0))
        node.heading_callback(Float32(data=1.0))
        assert node.yaw_deg == pytest.approx(-2.0)

        # Cross back 1 -> 359 (-2 deg CW == +2 deg CCW)
        node.heading_callback(Float32(data=359.0))
        assert node.yaw_deg == pytest.approx(0.0)

        # Cross -179 -> +179 (-2 deg CW == +2 deg CCW)
        node.prev_raw_heading = -179.0
        node.heading_callback(Float32(data=179.0))
        assert node.yaw_deg == pytest.approx(2.0)
    finally:
        node.destroy_node()


def test_heading_glitch_rejection():
    """Verify implausible single-step jumps (>= 45 deg) are ignored."""
    node = make_node()
    try:
        node.heading_callback(Float32(data=10.0))
        node.heading_callback(Float32(data=70.0))
        assert node.yaw_deg == pytest.approx(0.0)
    finally:
        node.destroy_node()


def test_imu_is_fresh():
    """Verify freshness depends on enable flag, receipt, and stale timeout."""
    node = make_node()
    try:
        assert not node.imu_is_fresh()
        node.heading_callback(Float32(data=5.0))
        assert node.imu_is_fresh()

        node.last_heading_time = time.monotonic() - (node.heading_stale_sec + 0.1)
        assert not node.imu_is_fresh()

        node.heading_callback(Float32(data=6.0))
        assert node.imu_is_fresh()
        node.heading_fusion_enable = False
        assert not node.imu_is_fresh()
    finally:
        node.destroy_node()


def test_pick_target_respects_corridor_alignment():
    """Verify IMU corridor weighting ignores reverse/corner holes and favors forward."""
    node = make_node()
    try:
        # Robot rotated -25 deg (pointing right).
        # Sensor -60 deg corresponds to -85 deg in corridor (outside max_corridor_deg=75).
        # Sensor +25 deg corresponds to 0 deg in corridor (straight toward finish).
        node.yaw_deg = -25.0
        scan = make_scan(
            lambda a: 1.15 if (abs(a - math.radians(-60.0)) < 0.25
                               or abs(a - math.radians(25.0)) < 0.25) else 0.45
        )
        node.scan_callback(scan)
        target_angle, _ = node.pick_target(imu_ok=True)
        corridor_target_deg = math.degrees(target_angle) + node.yaw_deg
        assert abs(corridor_target_deg) < 12.0
    finally:
        node.destroy_node()


def test_is_finish_wall_ahead_guards_and_fallback():
    """Verify side-wall rejection, angled finish acceptance, obstacle rejection, fallback."""
    node = make_node()
    try:
        # Case 1: Robot flipped 85 deg facing a close side wall -> IMU prevents false finish.
        node.yaw_deg = 85.0
        side_wall_scan = make_scan(lambda _: 0.28)
        node.scan_callback(side_wall_scan)
        assert not node.is_finish_wall_ahead(imu_ok=True, front_clear=0.28)
        # Without IMU, fallback triggers as before.
        assert node.is_finish_wall_ahead(imu_ok=False, front_clear=0.28)

        # Case 2: Narrow obstacle at 0.32m dead ahead, open corridor beside it -> not finish.
        node.yaw_deg = 0.0
        obstacle_scan = make_scan(lambda a: 0.32 if abs(a) <= math.radians(12.0) else 1.10)
        node.scan_callback(obstacle_scan)
        assert not node.is_finish_wall_ahead(imu_ok=True, front_clear=0.32)

        # Case 3: Angled approach (yaw = 20 deg) to real end wall 0.35m down corridor -> finish.
        yaw_deg = 20.0
        node.yaw_deg = yaw_deg
        end_wall_scan = make_scan(
            lambda a: min(
                1.5,
                0.35 / max(0.15, math.cos(a + math.radians(yaw_deg)))
            )
        )
        node.scan_callback(end_wall_scan)
        assert node.is_finish_wall_ahead(imu_ok=True, front_clear=0.35)
    finally:
        node.destroy_node()
