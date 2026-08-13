"""
Run timer: the stopwatch for one competition run.

This node measures one thing and drives nothing. It watches the run state a
driving node publishes and turns it into an elapsed time, so the run time is
visible on the car's OLED and in the logs without anyone holding a phone.

    STANDBY / ARMING   the clock sits at 00:00.0
    RUNNING            the clock starts, on the same tick the car does
    HOMING             the clock keeps going - homing is part of the run
    FINISHED           the clock freezes on the final time

The states themselves are parameters (start_states / stop_states /
reset_states), so the same node times any state machine that publishes a
String heartbeat. The defaults follow open_round_run's /open_round/state.

Minutes are the largest unit: /run_time_str is MM:SS.d, and the firmware
formats the OLED line the same way. A WRO run is a few minutes at most, so
hours would only ever be two dead characters on a 21 character display line.

Edge cases handled:
- Late start: a node launched into a run already in progress starts from the
  first RUNNING it sees and says so, rather than silently reporting a run
  time that is short by however long it was down.
- Clock steps: the ROS clock is the wall clock (or sim time) and either can
  jump. A stopwatch may never run backwards, so a backwards step rebases the
  start mark instead of rewinding the face.
- The run source disappearing: no state message for state_timeout_sec while
  the clock is running freezes it at the last state message rather than
  counting up forever against a node that is gone.
- Repeated states: only transitions act, so the 1 Hz heartbeat that repeats
  RUNNING cannot restart the clock, and a FINISHED that arrives twice cannot
  stop it twice.
- Restarts: a driving node coming back up publishes STANDBY, which re-arms
  the stopwatch for the next run - no need to restart this node between runs.

Run:
    ros2 run autonomy run_timer --ros-args \
        --params-file config/bot_config.yaml
Watch it:
    ros2 topic echo /run_time_str
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


def format_run_time(seconds):
    """
    Seconds as MM:SS.d, with minutes as the largest unit.

    Truncated rather than rounded, because a stopwatch reading 1.0 s at
    0.96 s is showing a tenth of a second that has not happened yet.
    """
    if not math.isfinite(seconds) or seconds <= 0.0:
        seconds = 0.0
    tenths = int(seconds * 10.0)
    minutes, rest = divmod(tenths, 600)
    return f'{minutes:02d}:{rest // 10:02d}.{rest % 10}'


class RunStopwatch:
    """
    Start / stop / reset and the elapsed time, with nothing ROS in it.

    The clock is injected (a callable returning seconds) so the whole state
    machine can be tested without waiting for real time to pass.
    """

    IDLE = 'IDLE'          # no run yet, or re-armed for the next one
    RUNNING = 'RUNNING'    # counting
    STOPPED = 'STOPPED'    # frozen on the final time

    def __init__(self, clock):
        self._clock = clock
        self._state = self.IDLE
        self._start_mark = 0.0
        self._elapsed = 0.0

    @property
    def state(self):
        return self._state

    @property
    def elapsed(self):
        """Seconds on the face as of the last read()."""
        return self._elapsed

    def read(self):
        """Advance the face to now (a no-op unless running) and return it."""
        if self._state == self.RUNNING:
            now = self._clock()
            elapsed = now - self._start_mark
            if elapsed < self._elapsed:
                # The clock stepped backwards. Hold the reading and re-hang
                # the start mark off the new clock, so the face carries on
                # from where it was instead of rewinding.
                self._start_mark = now - self._elapsed
            else:
                self._elapsed = elapsed
        return self._elapsed

    def start(self):
        """Begin timing. Ignored unless idle - one run is timed once."""
        if self._state != self.IDLE:
            return False
        self._state = self.RUNNING
        self._start_mark = self._clock()
        self._elapsed = 0.0
        return True

    def stop(self, at=None):
        """
        Freeze the face. Ignored unless running.

        `at` is the moment the run actually ended, for when that is known to
        be earlier than now - the run source went quiet and we only worked
        that out several ticks later. Winding the face back to it is
        deliberate: the run ended there, the rest was us finding out.
        """
        if self._state != self.RUNNING:
            return False
        if at is None:
            self.read()
        else:
            self._elapsed = max(0.0, at - self._start_mark)
        self._state = self.STOPPED
        return True

    def reset(self):
        """Back to zero and idle, ready for the next run."""
        if self._state == self.IDLE and self._elapsed == 0.0:
            return False
        self._state = self.IDLE
        self._elapsed = 0.0
        return True


class RunTimerNode(Node):

    def __init__(self, **kwargs):
        # kwargs go straight through to rclpy's Node (parameter_overrides and
        # friends), which is how the tests configure an instance without
        # going via the command line.
        super().__init__('run_timer', **kwargs)

        # ── What to follow ───────────────────────────────────────────
        # The run state heartbeat. open_round_run publishes this once a
        # second and again on every transition, so the clock starts on the
        # same tick the car does.
        self.declare_parameter('state_topic', '/open_round/state')
        # Entering any of these starts the clock, ...
        self.declare_parameter('start_states', ['RUNNING'])
        # ... any of these freezes it, ...
        self.declare_parameter('stop_states', ['FINISHED'])
        # ... and any of these arms it again for the next run. Anything else
        # (ARMING, HOMING) leaves the clock exactly as it is, which is why
        # HOMING keeps counting: it is part of the run.
        self.declare_parameter('reset_states', ['STANDBY'])

        # ── Output ───────────────────────────────────────────────────
        # 10 Hz is a tenth-of-a-second display updating smoothly, and it is
        # the rate mcu_bridge forwards to the OLED at.
        self.declare_parameter('publish_rate_hz', 10.0)
        # Seconds between progress lines while running. 0 disables them.
        self.declare_parameter('log_period_sec', 5.0)

        # ── Safety net ───────────────────────────────────────────────
        # Silence on state_topic this long while the clock is running means
        # the driving node died. Freeze rather than count up against a node
        # that is gone. 0 disables the check.
        self.declare_parameter('state_timeout_sec', 5.0)

        self.state_topic = str(self.get_parameter('state_topic').value)
        self.start_states = self._state_set('start_states')
        self.stop_states = self._state_set('stop_states')
        self.reset_states = self._state_set('reset_states')
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.state_timeout_sec = float(self.get_parameter('state_timeout_sec').value)

        # Fail fast on config that would leave the clock unable to run.
        if not self.start_states:
            raise ValueError('start_states must name at least one state, '
                             'otherwise the clock can never start')
        overlap = ((self.start_states & self.stop_states)
                   | (self.start_states & self.reset_states)
                   | (self.stop_states & self.reset_states))
        if overlap:
            raise ValueError(f'a state cannot mean two things at once: '
                             f'{sorted(overlap)} appears in more than one of '
                             f'start_states / stop_states / reset_states')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        if self.state_timeout_sec < 0.0:
            raise ValueError('state_timeout_sec cannot be negative')

        self.stopwatch = RunStopwatch(self.now_sec)
        self.last_run_state = None      # last state seen on state_topic
        self.last_state_time = None     # when it arrived, for the timeout

        self.timer_state_pub = self.create_publisher(String, '/run_timer_state', 10)
        self.time_pub = self.create_publisher(Float32, '/run_time', 10)
        self.text_pub = self.create_publisher(String, '/run_time_str', 10)

        self.state_sub = self.create_subscription(
            String, self.state_topic, self.run_state_callback, 10)

        self.create_timer(1.0 / publish_rate_hz, self.tick)

        self.get_logger().info(
            f'Run timer: following {self.state_topic}, starting on '
            f'{"/".join(sorted(self.start_states))} and stopping on '
            f'{"/".join(sorted(self.stop_states)) or "nothing"}. '
            f'Publishing /run_time, /run_time_str and /run_timer_state at '
            f'{publish_rate_hz:g} Hz.')

    def _state_set(self, param):
        """Read a parameter's state names, upper cased with blanks dropped."""
        values = self.get_parameter(param).value or []
        return {str(v).strip().upper() for v in values if str(v).strip()}

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ══════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════

    def run_state_callback(self, msg: String):
        """
        Act on transitions only.

        The heartbeat repeats the same state once a second, and a repeat is
        not a new event: without this, a RUNNING heartbeat would try to
        restart the clock every second of the run.
        """
        state = msg.data.strip().upper()
        self.last_state_time = self.now_sec()

        if state == self.last_run_state:
            return
        previous, self.last_run_state = self.last_run_state, state

        if state in self.start_states:
            if self.stopwatch.start():
                if previous is None:
                    self.get_logger().warn(
                        f'First {self.state_topic} message was already {state} — '
                        f'the run was underway before this node was. Timing from '
                        f'now, so the run time is a lower bound.')
                else:
                    self.get_logger().info(f'{state} — clock started.')
            else:
                self.get_logger().info(
                    f'{state} again while {self.stopwatch.state}, clock left alone.')
        elif state in self.stop_states:
            if self.stopwatch.stop():
                self.get_logger().info(
                    f'{state} — clock stopped at '
                    f'{format_run_time(self.stopwatch.elapsed)} '
                    f'({self.stopwatch.elapsed:.2f} s).')
        elif state in self.reset_states:
            if self.stopwatch.reset():
                self.get_logger().info(f'{state} — clock re-armed for the next run.')
        else:
            # ARMING, HOMING and anything else: part of the run, not an event.
            self.get_logger().debug(f'{state} — clock left as it is.')

    # ══════════════════════════════════════════════════════════════════
    # Publish Loop
    # ══════════════════════════════════════════════════════════════════

    def tick(self):
        self.stopwatch.read()
        self.check_state_timeout()

        elapsed = self.stopwatch.elapsed
        text = format_run_time(elapsed)

        # State goes out first, so a consumer that reads the pair in arrival
        # order can never latch the final time as if it were still running.
        self.timer_state_pub.publish(String(data=self.stopwatch.state))
        self.time_pub.publish(Float32(data=float(elapsed)))
        self.text_pub.publish(String(data=text))

        if self.stopwatch.state == RunStopwatch.RUNNING and self.log_period_sec > 0.0:
            self.get_logger().info(
                f'Run time {text}', throttle_duration_sec=self.log_period_sec)

    def check_state_timeout(self):
        """Freeze the clock if the node publishing the run state has gone."""
        if self.state_timeout_sec <= 0.0:
            return
        if self.stopwatch.state != RunStopwatch.RUNNING:
            return
        if self.last_state_time is None:
            return

        silent = self.now_sec() - self.last_state_time
        if silent < self.state_timeout_sec:
            return

        self.stopwatch.stop(at=self.last_state_time)
        self.get_logger().error(
            f'Nothing on {self.state_topic} for {silent:.1f} s with the clock '
            f'running — the run source is gone. Frozen at the last state '
            f'message: {format_run_time(self.stopwatch.elapsed)}.')


def main(args=None):
    rclpy.init(args=args)
    run_timer_node = RunTimerNode()
    try:
        rclpy.spin(run_timer_node)
    except KeyboardInterrupt:
        pass
    finally:
        run_timer_node.get_logger().info(
            f'Stopping — last run time '
            f'{format_run_time(run_timer_node.stopwatch.elapsed)}')
        run_timer_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
