"""
Tests for the run stopwatch.

The state machine and the formatter are pure and take an injected clock, so
everything below runs instantly and deterministically - no sleeping, no
waiting for a real second to pass. The last section spins the actual node
against a fake state publisher to check the ROS wiring.

    python3 -m pytest ros2_ws/autonomy/test/test_run_timer.py -v
"""

import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, String

from autonomy.run_timer import RunStopwatch, RunTimerNode, format_run_time

# Somewhere nothing else lives, so a car running the real stack on this
# machine cannot join in. Any value in 0-101 works for the default DDS.
TEST_DOMAIN_ID = '77'


class FakeClock:
    """A clock that only moves when the test says so."""

    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds
        return self.now


# ══════════════════════════════════════════════════════════════════════
# format_run_time
# ══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('seconds,expected', [
    (0.0, '00:00.0'),
    (0.09, '00:00.0'),
    (0.1, '00:00.1'),
    (1.25, '00:01.2'),        # truncated, not rounded up to .3
    (9.99, '00:09.9'),
    (59.9, '00:59.9'),
    (60.0, '01:00.0'),
    (65.4, '01:05.4'),
    (599.95, '09:59.9'),
    (600.0, '10:00.0'),
    (3600.0, '60:00.0'),      # minutes stay the largest unit, no hours field
    (6000.0, '100:00.0'),     # and they are not clamped either
])
def test_format_run_time(seconds, expected):
    assert format_run_time(seconds) == expected


@pytest.mark.parametrize('seconds', [-0.0, -1.0, -1e9, float('nan'),
                                     float('inf'), float('-inf')])
def test_format_run_time_rejects_nonsense(seconds):
    """A bad number shows a zeroed clock, never a crash or a garbled line."""
    assert format_run_time(seconds) == '00:00.0'


def test_format_run_time_never_shows_60_seconds():
    """Every tenth of the first two minutes formats to a legal clock face."""
    for tenth in range(0, 1200):
        text = format_run_time(tenth / 10.0)
        minutes, rest = text.split(':')
        seconds, frac = rest.split('.')
        assert 0 <= int(seconds) <= 59, text
        assert 0 <= int(frac) <= 9, text
        assert int(minutes) == tenth // 600, text


# ══════════════════════════════════════════════════════════════════════
# RunStopwatch
# ══════════════════════════════════════════════════════════════════════

def test_starts_idle_at_zero():
    watch = RunStopwatch(FakeClock())
    assert watch.state == RunStopwatch.IDLE
    assert watch.elapsed == 0.0
    assert watch.read() == 0.0


def test_counts_while_running():
    clock = FakeClock()
    watch = RunStopwatch(clock)

    assert watch.start() is True
    assert watch.state == RunStopwatch.RUNNING
    assert watch.read() == 0.0

    clock.advance(12.5)
    assert watch.read() == pytest.approx(12.5)
    clock.advance(30.0)
    assert watch.read() == pytest.approx(42.5)


def test_idle_clock_does_not_count():
    """Standby and arming must leave the face at zero however long they take."""
    clock = FakeClock()
    watch = RunStopwatch(clock)
    clock.advance(3600.0)
    assert watch.read() == 0.0

    # ...and the run that follows still starts from zero.
    watch.start()
    clock.advance(1.0)
    assert watch.read() == pytest.approx(1.0)


def test_start_is_ignored_while_already_running():
    """A repeated RUNNING must not restart the clock mid run."""
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    clock.advance(20.0)
    watch.read()

    assert watch.start() is False
    clock.advance(5.0)
    assert watch.read() == pytest.approx(25.0)


def test_stop_freezes_the_face():
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    clock.advance(90.0)

    assert watch.stop() is True
    assert watch.state == RunStopwatch.STOPPED
    assert watch.elapsed == pytest.approx(90.0)

    clock.advance(600.0)
    assert watch.read() == pytest.approx(90.0)
    assert watch.elapsed == pytest.approx(90.0)


def test_stop_is_ignored_unless_running():
    clock = FakeClock()
    watch = RunStopwatch(clock)
    assert watch.stop() is False        # never started

    watch.start()
    clock.advance(10.0)
    assert watch.stop() is True
    clock.advance(10.0)
    assert watch.stop() is False        # a second FINISHED cannot move it
    assert watch.elapsed == pytest.approx(10.0)


def test_stop_at_an_earlier_moment_winds_back():
    """
    The clock has to say when the run ended, not when we noticed.

    The state source went quiet at t+40 but we only worked that out at t+45.
    The run ended at 40.
    """
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    ended = clock.advance(40.0)
    clock.advance(5.0)
    watch.read()                        # the face has already run past the end

    watch.stop(at=ended)
    assert watch.elapsed == pytest.approx(40.0)


def test_stop_at_before_the_start_cannot_go_negative():
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    clock.advance(10.0)
    watch.stop(at=clock.now - 500.0)
    assert watch.elapsed == 0.0


def test_reset_rearms_for_the_next_run():
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    clock.advance(75.0)
    watch.stop()

    assert watch.reset() is True
    assert watch.state == RunStopwatch.IDLE
    assert watch.elapsed == 0.0
    assert watch.reset() is False       # already armed, nothing to do

    watch.start()
    clock.advance(3.0)
    assert watch.read() == pytest.approx(3.0)


def test_reset_mid_run_abandons_the_run():
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    clock.advance(30.0)
    watch.read()

    assert watch.reset() is True
    assert watch.state == RunStopwatch.IDLE
    assert watch.elapsed == 0.0


def test_clock_stepping_backwards_never_rewinds_the_face():
    """NTP correcting a Pi with no RTC must not make the run time shrink."""
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    clock.advance(50.0)
    assert watch.read() == pytest.approx(50.0)

    clock.advance(-20.0)                # the wall clock jumps back
    assert watch.read() == pytest.approx(50.0)

    clock.advance(5.0)                  # and carries on from where it was
    assert watch.read() == pytest.approx(55.0)


def test_clock_stepping_forward_is_taken_at_face_value():
    """A forward step is indistinguishable from time really passing."""
    clock = FakeClock()
    watch = RunStopwatch(clock)
    watch.start()
    clock.advance(3600.0)
    assert watch.read() == pytest.approx(3600.0)


# ══════════════════════════════════════════════════════════════════════
# The node
# ══════════════════════════════════════════════════════════════════════

class TimerHarness:
    """A run_timer node plus a fake state publisher, spun by hand."""

    def __init__(self, *overrides):
        self.timer = RunTimerNode(parameter_overrides=list(overrides))
        self.peer = Node('test_peer')
        self.state_pub = self.peer.create_publisher(String, '/open_round/state', 10)
        self.seen_states = []
        self.seen_times = []
        self.seen_texts = []
        self.peer.create_subscription(
            String, '/run_timer_state', lambda m: self.seen_states.append(m.data), 10)
        self.peer.create_subscription(
            Float32, '/run_time', lambda m: self.seen_times.append(m.data), 10)
        self.peer.create_subscription(
            String, '/run_time_str', lambda m: self.seen_texts.append(m.data), 10)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.timer)
        self.executor.add_node(self.peer)
        self.spin(0.5)  # let discovery settle before anything is published

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def publish_state(self, state, settle=0.35):
        self.state_pub.publish(String(data=state))
        self.spin(settle)

    def close(self):
        self.executor.shutdown()
        self.timer.destroy_node()
        self.peer.destroy_node()


@pytest.fixture
def ros():
    """
    Bring rclpy up on a domain of its own.

    The topic names below are the real ones, so a stack running on the same
    machine - a car sitting in standby while someone runs the tests over ssh -
    would otherwise feed its own /open_round/state into these nodes and fail
    them for no reason.
    """
    previous = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN_ID
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()
        if previous is None:
            del os.environ['ROS_DOMAIN_ID']
        else:
            os.environ['ROS_DOMAIN_ID'] = previous


def test_node_times_a_whole_run(ros):
    """STANDBY -> ARMING -> RUNNING -> HOMING -> FINISHED, end to end."""
    harness = TimerHarness()
    try:
        harness.publish_state('STANDBY')
        harness.publish_state('ARMING', settle=0.5)
        assert harness.timer.stopwatch.state == RunStopwatch.IDLE
        assert harness.timer.stopwatch.elapsed == 0.0, 'arming must not count'

        harness.publish_state('RUNNING')
        assert harness.timer.stopwatch.state == RunStopwatch.RUNNING
        running_at = harness.timer.stopwatch.elapsed
        assert running_at > 0.0

        # Homing is part of the run, so the clock keeps going through it.
        harness.publish_state('HOMING', settle=0.5)
        assert harness.timer.stopwatch.state == RunStopwatch.RUNNING
        homing_at = harness.timer.stopwatch.elapsed
        assert homing_at > running_at

        harness.publish_state('FINISHED')
        final = harness.timer.stopwatch.elapsed
        assert harness.timer.stopwatch.state == RunStopwatch.STOPPED
        assert final >= homing_at

        # and it stays frozen from there
        harness.spin(0.5)
        assert harness.timer.stopwatch.elapsed == pytest.approx(final)
        assert harness.seen_states[-1] == 'STOPPED'
        assert harness.seen_times[-1] == pytest.approx(final, abs=1e-6)
    finally:
        harness.close()


def test_node_publishes_all_three_topics(ros):
    harness = TimerHarness()
    try:
        harness.publish_state('RUNNING', settle=0.6)

        assert harness.seen_states, '/run_timer_state never published'
        assert harness.seen_times, '/run_time never published'
        assert harness.seen_texts, '/run_time_str never published'
        assert harness.seen_states[-1] == 'RUNNING'
        assert harness.seen_texts[-1].startswith('00:00.')
        # the published seconds and the published text agree
        assert format_run_time(harness.seen_times[-1]) == harness.seen_texts[-1]
        # and the clock only ever moves forwards on the wire
        assert harness.seen_times == sorted(harness.seen_times)
    finally:
        harness.close()


def test_node_rearms_when_the_driving_node_restarts(ros):
    harness = TimerHarness()
    try:
        harness.publish_state('RUNNING', settle=0.5)
        harness.publish_state('FINISHED')
        assert harness.timer.stopwatch.elapsed > 0.0

        # open_round_run comes back up and announces standby
        harness.publish_state('STANDBY')
        assert harness.timer.stopwatch.state == RunStopwatch.IDLE
        assert harness.timer.stopwatch.elapsed == 0.0

        harness.publish_state('RUNNING', settle=0.4)
        assert harness.timer.stopwatch.state == RunStopwatch.RUNNING
    finally:
        harness.close()


def test_node_heartbeat_does_not_restart_the_clock(ros):
    """open_round_run repeats RUNNING every second; that must be a no-op."""
    harness = TimerHarness()
    try:
        harness.publish_state('RUNNING', settle=0.5)
        first = harness.timer.stopwatch.elapsed
        for _ in range(3):
            harness.publish_state('RUNNING', settle=0.2)
        assert harness.timer.stopwatch.elapsed > first
    finally:
        harness.close()


def test_node_freezes_when_the_state_source_disappears(ros):
    """
    A driving node dying mid run stops the clock where it last spoke.

    Counting up against a node that no longer exists would turn a crashed run
    into a plausible looking run time.
    """
    timeout = 0.6
    harness = TimerHarness(Parameter('state_timeout_sec', value=timeout))
    try:
        harness.publish_state('RUNNING', settle=0.3)
        # HOMING is the last heartbeat that gets out before the node dies, so
        # it lands somewhere between these two readings of the clock.
        before_last = harness.timer.stopwatch.elapsed
        harness.publish_state('HOMING', settle=0.3)
        after_last = harness.timer.stopwatch.elapsed
        assert harness.timer.stopwatch.state == RunStopwatch.RUNNING

        harness.spin(1.5)  # nothing more published: open_round_run is gone
        assert harness.timer.stopwatch.state == RunStopwatch.STOPPED

        frozen = harness.timer.stopwatch.elapsed
        assert before_last <= frozen <= after_last + 0.05, (
            f'froze at {frozen:.2f} s, outside the window the last state '
            f'message landed in ({before_last:.2f}-{after_last:.2f} s)')
        # the timeout only fires `timeout` seconds after that last message, so
        # this is what tells us the clock wound back instead of freezing late
        assert frozen < after_last + timeout - 0.2, (
            'froze when it noticed, not when the run actually ended')
        assert harness.seen_states[-1] == 'STOPPED'
    finally:
        harness.close()


def test_node_state_timeout_can_be_switched_off(ros):
    harness = TimerHarness(Parameter('state_timeout_sec', value=0.0))
    try:
        harness.publish_state('RUNNING', settle=0.2)
        harness.spin(1.0)
        assert harness.timer.stopwatch.state == RunStopwatch.RUNNING
    finally:
        harness.close()


def test_node_rejects_contradictory_state_config(ros):
    """A state that means both start and stop is a config bug, not a default."""
    with pytest.raises(ValueError):
        TimerHarness(Parameter('stop_states', Parameter.Type.STRING_ARRAY, ['RUNNING']))


def test_node_rejects_an_unstartable_config(ros):
    """Blank entries are dropped, so a list of them is no list at all."""
    with pytest.raises(ValueError):
        TimerHarness(Parameter('start_states', Parameter.Type.STRING_ARRAY, ['', '  ']))
