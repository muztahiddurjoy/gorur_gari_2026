"""
Score a run from the simulator's ground truth. The robot sees none of this.

Reads the layout spawn_boxes wrote and follows /ground_truth until /nav/state says
FINISHED, or the time limit passes, then prints a verdict and optionally writes it as
JSON. A run passes if the base centre stopped inside the finish zone, every box was
passed on its legal side, and the footprint touched neither a box nor a wall.
"""
import argparse
import json
import math
import sys

from fastrobox_sim.box_layout import (
    Box, footprint_gap, RED, ROBOT_LENGTH, ROBOT_WIDTH, TRACK_HALF_WIDTH,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import yaml

FINISH_LINE_X = 2.8
TRACK_LENGTH = 3.0


def wall_gap(x, y, yaw):
    """Return the gap between a robot footprint at (x, y, yaw) and the nearest wall."""
    reach_x = ROBOT_LENGTH / 2 * abs(math.cos(yaw)) + ROBOT_WIDTH / 2 * abs(math.sin(yaw))
    reach_y = ROBOT_LENGTH / 2 * abs(math.sin(yaw)) + ROBOT_WIDTH / 2 * abs(math.cos(yaw))
    return min(x - reach_x, TRACK_LENGTH - x - reach_x, TRACK_HALF_WIDTH - abs(y) - reach_y)


class Referee(Node):

    def __init__(self, layout, time_limit):
        super().__init__('referee')
        self.seed = layout.get('seed')
        self.boxes = [Box(item['color'], item['x'], item['y']) for item in layout['boxes']]
        self.time_limit = time_limit
        self.start_time = None
        self.pose = None
        self.nav_state = None
        self.done = False
        self.verdict = None
        self.passed_at = [None] * len(self.boxes)
        self.min_box_gap = [math.inf] * len(self.boxes)
        self.min_wall_gap = math.inf
        self.max_abs_y = 0.0
        self.create_subscription(Odometry, '/ground_truth', self.on_truth, 50)
        self.create_subscription(
            String, '/nav/state', self.on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_state(self, message):
        self.nav_state = message.data
        if message.data == 'FINISHED':
            self.finish()

    def on_truth(self, message):
        if self.done:
            return
        now = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = now
        pose = message.pose.pose
        x, y = pose.position.x, pose.position.y
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        previous_x = self.pose[0] if self.pose else x
        self.pose = (x, y, yaw, now)
        self.min_wall_gap = min(self.min_wall_gap, wall_gap(x, y, yaw))
        self.max_abs_y = max(self.max_abs_y, abs(y))
        for index, box in enumerate(self.boxes):
            self.min_box_gap[index] = min(self.min_box_gap[index], footprint_gap(box, x, y, yaw))
            if self.passed_at[index] is None and previous_x < box.x <= x:
                self.passed_at[index] = y
        if now - self.start_time > self.time_limit:
            self.finish()

    def finish(self):
        if self.done or self.pose is None:
            return
        self.done = True
        x, y, yaw, now = self.pose
        boxes = []
        for box, passed_y, gap in zip(self.boxes, self.passed_at, self.min_box_gap):
            wanted = 'left' if box.color == RED else 'right'
            side = None if passed_y is None else ('left' if passed_y > box.y else 'right')
            boxes.append({
                'color': box.color, 'x': box.x, 'y': box.y, 'wanted': wanted, 'side': side,
                'legal': side == wanted, 'gap': round(gap, 3),
            })
        finished = self.nav_state == 'FINISHED' and FINISH_LINE_X <= x <= TRACK_LENGTH
        touched_box = any(item['gap'] <= 0.0 for item in boxes)
        touched_wall = self.min_wall_gap <= 0.0
        self.verdict = {
            'seed': self.seed,
            'passed': (finished and not touched_box and not touched_wall
                       and all(item['legal'] for item in boxes)),
            'finished': finished,
            'time_s': round(now - self.start_time, 1),
            'final_x': round(x, 3), 'final_y': round(y, 3),
            'final_yaw_deg': round(math.degrees(yaw), 1),
            'min_box_gap': round(min(self.min_box_gap), 3),
            'min_wall_gap': round(self.min_wall_gap, 3),
            'illegal_boxes': sum(1 for item in boxes if not item['legal']),
            'touched_box': touched_box,
            'touched_wall': touched_wall,
            'boxes': boxes,
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--layout-file', default='/tmp/fastrobox_layout.yaml')
    parser.add_argument('--result-file', default=None)
    parser.add_argument('--time-limit', type=float, default=120.0, help='Sim seconds.')
    args, ros_args = parser.parse_known_args(argv)

    with open(args.layout_file) as handle:
        layout = yaml.safe_load(handle)

    rclpy.init(args=ros_args)
    referee = Referee(layout, args.time_limit)
    while rclpy.ok() and not referee.done:
        rclpy.spin_once(referee, timeout_sec=0.5)
    if referee.verdict is None:
        return 1
    verdict = referee.verdict
    if args.result_file:
        with open(args.result_file, 'w') as handle:
            json.dump(verdict, handle, indent=2)
    print(f"referee: {'PASS' if verdict['passed'] else 'FAIL'} seed={verdict['seed']} "
          f"time={verdict['time_s']}s final=({verdict['final_x']}, {verdict['final_y']}) "
          f"box_gap={verdict['min_box_gap']} wall_gap={verdict['min_wall_gap']}", flush=True)
    for item in verdict['boxes']:
        print(f"referee:   {item['color']:5} x={item['x']:.2f} y={item['y']:+.2f} "
              f"passed {item['side']} (wanted {item['wanted']}), gap {item['gap']}", flush=True)
    referee.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
