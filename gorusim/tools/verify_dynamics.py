#!/usr/bin/env python3
"""
verify_dynamics - drive the simulated car and check it against the numbers it was built
from. Run it with the simulator up and nothing else driving:

    ros2 launch gorusim sim.launch.py gui:=false rviz:=false build_track:=false \\
        world:=flatWorld
    python3 gorusim/tools/verify_dynamics.py

Exits non-zero if a check fails. Ground truth is the simulator's own odometry.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32, Int32, UInt8

from gorusim.linkage import SteeringLinkage
from gorusim.sim_mcu import horn_from_wire

METRES_PER_TICK = 0.057 * math.pi * 1.0 / 400  # what vector_odom is launched with
WHEELBASE = 0.140


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__('verify_dynamics')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose = None
        self.speed = 0.0
        self.ticks = None
        self.heading = None
        self.wire = None
        self.joints = {}
        self.scan = None
        self.create_subscription(Odometry, '/gorubot/ground_truth', self.on_odom, 10)
        self.create_subscription(Int32, '/encoder/count', self.on_ticks, 10)
        self.create_subscription(Float32, '/heading', self.on_heading, 10)
        self.create_subscription(UInt8, '/steering_angle', self.on_wire, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def on_ticks(self, msg):
        self.ticks = msg.data

    def on_heading(self, msg):
        self.heading = msg.data

    def on_wire(self, msg):
        self.wire = msg.data

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_scan(self, msg):
        self.scan = msg

    def drive(self, linear, angular, seconds, each=None):
        end = time.time() + seconds
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        while time.time() < end:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
            if each:
                each()

    def stop(self, seconds=2.0):
        self.drive(0.0, 0.0, seconds)


class Report:
    def __init__(self):
        self.failed = 0

    def check(self, name, value, low, high, unit=''):
        ok = low <= value <= high
        self.failed += 0 if ok else 1
        print(f'  [{"PASS" if ok else "FAIL"}] {name}: {value:.3f}{unit}  '
              f'(expected {low:g} .. {high:g})')


def circle_radius(points):
    """Least squares circle through (x, y) points (Kasa fit)."""
    n = len(points)
    sx = sum(p[0] for p in points) / n
    sy = sum(p[1] for p in points) / n
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for x, y in points:
        u, v = x - sx, y - sy
        suu += u * u
        svv += v * v
        suv += u * v
        suuu += u ** 3
        svvv += v ** 3
        suvv += u * v * v
        svuu += v * u * u
    det = suu * svv - suv * suv
    uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / (2.0 * det)
    vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / (2.0 * det)
    return math.sqrt(uc * uc + vc * vc + (suu + svv) / n)


def main():
    rclpy.init()
    node = Probe()
    report = Report()
    linkage = SteeringLinkage()

    print('Waiting for the car...')
    deadline = time.time() + 30.0
    while time.time() < deadline and (node.pose is None or node.ticks is None
                                      or node.heading is None or node.scan is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose is None or node.ticks is None or node.scan is None:
        print('No telemetry: is sim.launch.py up?')
        return 2
    node.stop(1.0)

    print('At rest')
    report.check('sits on its tyres, z', node.pose[2], -0.002, 0.003, ' m')

    print('Self-returns behind the scanner (standoffs, Pi connectors)')
    scan = node.scan
    rear = [r for i, r in enumerate(scan.ranges)
            if abs(scan.angle_min + i * scan.angle_increment) > math.radians(120)
            and scan.range_min < r < 0.30]
    front = [r for i, r in enumerate(scan.ranges)
             if abs(scan.angle_min + i * scan.angle_increment) < math.radians(80)
             and scan.range_min < r < 0.10]
    print(f'    rear arc rays under 0.30 m: {len(rear)}, '
          f'front arc rays under 0.10 m: {len(front)}')
    report.check('nothing of the car in the front +-80 deg', len(front), 0, 0)

    print('Below the breakaway throttle')
    # Past the launch boost window so the doubled command is gone.
    start = node.ticks
    node.drive(0.10, 0.0, 3.0)
    report.check('wire 10 stays stalled, ticks', abs(node.ticks - start), 0, 5)
    node.stop(1.0)

    print('The breakaway, then rolling below it')
    # mcu_bridge doubles this for the first second, which breaks the car free, and it
    # has to keep rolling at the plain command afterwards: bot_config's creep speed.
    start = node.ticks
    node.drive(0.40, 0.0, 2.5)
    moved_ticks = node.ticks - start
    report.check('rolls on at wire 40 after the launch boost, speed',
                 node.speed, 0.10, 1.0, ' m/s')
    print(f'    {moved_ticks} ticks')
    node.stop(1.5)

    print('Straight line at the lapping throttle, drive_speed 1.25')
    x0, y0 = node.pose[0], node.pose[1]
    tick0 = node.ticks
    speeds = []
    node.drive(1.25, 0.0, 2.5, each=lambda: speeds.append(node.speed))
    top = max(speeds)
    # Compare at rest: telemetry is up to 50 ms old, which at speed is 2-3% of this run.
    node.stop(2.5)
    moved = math.hypot(node.pose[0] - x0, node.pose[1] - y0)
    ticks = node.ticks - tick0
    print(f'    {moved:.3f} m, top speed {top:.2f} m/s, {ticks} ticks')
    report.check('odometry reads true: ticks x 0.549 mm / truth',
                 ticks * METRES_PER_TICK / moved, 0.98, 1.02)
    report.check('top speed under the 50% duty ceiling', top, 0.3, 1.31, ' m/s')

    for name, command, wire in (('left', 0.5, 135), ('right', -0.5, 45)):
        horn = horn_from_wire(wire)
        left, right = linkage.angles(math.radians(horn))
        mean = (left + right) / 2.0
        expected = WHEELBASE / math.tan(abs(mean))
        print(f'{name} turn, angular.z {command:+.1f}: wire {wire}, horn {horn:+.0f} deg, '
              f'wheels {math.degrees(left):.1f}/{math.degrees(right):.1f} deg')
        points = []

        def sample():
            if node.speed > 0.05:
                points.append(node.pose[:2])

        # The lapping throttle. A solid axle has to scrub to get round at all,
        # which is why the real stack raises the throttle in turns, not lowers it.
        node.drive(1.25, command, 1.0)
        report.check('steering_angle telemetry', node.wire, wire - 1, wire + 1)
        report.check('kingpin joint (left)', node.joints.get('front_left_kingpin', 99.0),
                     left - 0.06, left + 0.06, ' rad')
        yaw_start = node.pose[3]
        node.drive(1.25, command, 0.5, each=sample)
        step = node.pose[3] - yaw_start
        turned = math.atan2(math.sin(step), math.cos(step))
        # REP-103: + is counter-clockwise, which is what angular.z > 0 has to produce.
        report.check('yaw goes the commanded way', math.copysign(1.0, turned),
                     *((1.0, 1.0) if command > 0 else (-1.0, -1.0)))
        node.drive(1.25, command, 4.5, each=sample)
        radius = circle_radius(points) if len(points) > 20 else float('nan')
        print(f'    radius {radius:.3f} m, kinematic {expected:.3f} m')
        # A solid axle scrubs, so the real arc runs wider than the kinematic one.
        report.check('radius against kinematics (solid axle runs wide)',
                     radius / expected, 0.95, 1.6)
        node.stop(2.0)

    print('Full travel')
    for name, command in (('left', 1.0), ('right', -1.0)):
        node.drive(1.25, command, 1.0)
        wire = node.wire
        left, right = (node.joints.get('front_left_kingpin', 0.0),
                       node.joints.get('front_right_kingpin', 0.0))
        expect_left, expect_right = linkage.angles(math.radians(horn_from_wire(wire)))
        print(f'    {name}: wire {wire}, wheels {math.degrees(left):.1f}/'
              f'{math.degrees(right):.1f} deg, linkage says '
              f'{math.degrees(expect_left):.1f}/{math.degrees(expect_right):.1f}')
        report.check(f'{name} lock, left kingpin', left,
                     expect_left - 0.08, expect_left + 0.08, ' rad')
        node.stop(1.5)

    print(f'{report.failed} check(s) failed.' if report.failed else 'All checks passed.')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
