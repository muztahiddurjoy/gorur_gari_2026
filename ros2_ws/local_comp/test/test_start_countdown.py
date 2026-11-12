"""The start button gate of both round drivers: press edge -> start_delay_sec -> drive."""
import time

from geometry_msgs.msg import Twist
from local_comp.obstacle_round import ObstacleRound
from local_comp.open_round import OpenRound
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool

DRIVERS = [OpenRound, ObstacleRound]


@pytest.fixture
def ros():
    """Call with a start delay to bring rclpy up with it set for every node."""
    started = []

    def init(delay):
        rclpy.init(args=['--ros-args', '-p', f'start_delay_sec:={float(delay)}'])
        started.append(True)

    yield init
    if started:
        rclpy.shutdown()


def make(ros, driver, delay=5.0):
    ros(delay)
    return driver()


def press(node, *levels):
    for level in levels:
        node.button_callback(Bool(data=level))


@pytest.mark.parametrize('driver', DRIVERS)
def test_press_arms_a_one_shot_timer_of_start_delay(ros, driver):
    node = make(ros, driver)
    press(node, True)
    assert node.state == 'COUNTDOWN'
    assert node.start_timer.timer_period_ns == 5_000_000_000
    node.start_run()
    assert node.state == 'RUNNING'
    assert node.start_timer.is_canceled()
    node.destroy_node()


@pytest.mark.parametrize('driver', DRIVERS)
def test_holding_the_button_is_one_press(ros, driver):
    node = make(ros, driver)
    press(node, True)
    first = node.start_timer
    press(node, True, True, True)  # the MCU repeats True while held
    assert node.start_timer is first
    node.destroy_node()


@pytest.mark.parametrize('driver', DRIVERS)
def test_a_new_press_restarts_the_countdown_like_the_led(ros, driver):
    node = make(ros, driver)
    press(node, True, False)
    first = node.start_timer
    press(node, True)
    assert node.state == 'COUNTDOWN'
    assert node.start_timer is not first
    node.destroy_node()


@pytest.mark.parametrize('driver', DRIVERS)
def test_a_press_while_running_is_ignored(ros, driver):
    node = make(ros, driver)
    press(node, True, False)
    node.start_run()
    timer = node.start_timer
    press(node, True)
    assert node.state == 'RUNNING'
    assert node.start_timer is timer
    node.destroy_node()


@pytest.mark.parametrize('driver', DRIVERS)
def test_a_press_after_finishing_starts_again(ros, driver):
    node = make(ros, driver)
    node.state = 'FINISHED'
    press(node, True)
    assert node.state == 'COUNTDOWN'
    node.destroy_node()


@pytest.mark.parametrize('driver', DRIVERS)
def test_first_drive_command_leaves_start_delay_after_the_press(ros, driver):
    delay = 0.4
    node = make(ros, driver, delay)
    stamps = []
    probe = rclpy.create_node('cmd_vel_probe')
    probe.create_subscription(Twist, '/cmd_vel', lambda _: stamps.append(time.monotonic()), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)

    pressed_at = time.monotonic()
    press(node, True)
    while not stamps and time.monotonic() - pressed_at < delay + 1.0:
        executor.spin_once(timeout_sec=0.005)

    assert node.state == 'RUNNING'
    assert stamps, 'no /cmd_vel after the countdown'
    # first command comes from start_run() itself, not the next 20 Hz tick
    assert stamps[0] - pressed_at == pytest.approx(delay, abs=0.02)
    executor.shutdown()
    probe.destroy_node()
    node.destroy_node()


@pytest.mark.parametrize('driver', DRIVERS)
def test_nothing_is_published_during_the_countdown(ros, driver):
    node = make(ros, driver, 0.3)
    stamps = []
    probe = rclpy.create_node('cmd_vel_probe')
    probe.create_subscription(Twist, '/cmd_vel', lambda _: stamps.append(time.monotonic()), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)

    press(node, True)
    end = time.monotonic() + 0.25
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.005)
    assert node.state == 'COUNTDOWN'
    assert not stamps
    executor.shutdown()
    probe.destroy_node()
    node.destroy_node()


@pytest.mark.parametrize('driver', DRIVERS)
def test_start_delay_must_be_positive(ros, driver):
    with pytest.raises(ValueError):
        make(ros, driver, 0.0)
