#!/usr/bin/env python3
"""
Drive to the finish, avoiding boxes by colour: red turns left, green turns right.

    /camera/image_raw --> find boxes and the finish --> where they are on the floor
    /odom ------------------------------------------> where the robot is
                                                            |
                                  which lane the boxes allow, steer for it --> /cmd_vel

Default state FORWARD. A red box sends the robot left of it (TURN_LEFT), a green box
right of it (TURN_RIGHT); once it holds a lane that clears the box it straightens up
and goes on along the track. The node stops in FINISHED when the base is in the finish
zone.

/odom is not in the original brief, but the camera cannot do this alone. Turning away
from a box and "resuming forward" on the new heading runs into a side wall; heading
back to the track's direction needs a measure of the turn. A box also leaves the frame
while the robot is still alongside it, so its position must be remembered.

Vision and control are plain classes with no ROS in them, so they can be tested without
a simulator. Robot frame: x forward, y left. The robot must start facing along the
track, on its centre line, since the odom frame then doubles as the track's frame.
"""
from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

RED = 'red'
GREEN = 'green'

# OpenCV HSV: H 0..179, S and V 0..255. Measured on rendered frames of the arena:
#   box faces       S 255, V 135-255, H 0 or 60
#   finish checks   S 162-165, V 100-120, H 70-71
#   start zone      S about 70
# Saturation is what separates a green box from the finish zone.
RED_LOWER_1, RED_UPPER_1 = (0, 210, 90), (8, 255, 255)
RED_LOWER_2, RED_UPPER_2 = (172, 210, 90), (179, 255, 255)
GREEN_LOWER, GREEN_UPPER = (50, 210, 90), (68, 255, 255)
FINISH_LOWER, FINISH_UPPER = (65, 130, 60), (80, 200, 150)

MIN_BOX_AREA_PX = 40
# Below this saturation the patch under a blob counts as bare floor.
FLOOR_MAX_SATURATION = 60
# Rows at the bottom of a blob used for its position. Further boxes of the same colour
# can merge into a blob above it, but the bottom always belongs to the nearest box.
BOTTOM_STRIP_ROWS = 6
# Share of a row that must be finish checks for the row to count as the finish zone.
FINISH_MIN_ROW_COVERAGE = 0.12


@dataclass(frozen=True)
class CameraModel:
    """A level-mounted, downward-tilted pinhole camera. Defaults match the xacro."""

    width: int = 640
    height: int = 480
    hfov: float = 1.3963
    pitch: float = 0.35
    x: float = 0.08
    z: float = 0.16

    @property
    def focal(self) -> float:
        return (self.width / 2) / math.tan(self.hfov / 2)

    def floor_point(self, column: float, row: float) -> Optional[Tuple[float, float]]:
        """Return the floor point (forward, left) at a pixel, None at or above the horizon."""
        ray_left = (self.width / 2 - column) / self.focal
        ray_up = (self.height / 2 - row) / self.focal
        cos_pitch, sin_pitch = math.cos(self.pitch), math.sin(self.pitch)
        along = cos_pitch + ray_up * sin_pitch
        down = sin_pitch - ray_up * cos_pitch
        if down <= 1e-6:
            return None
        scale = self.z / down
        return self.x + along * scale, ray_left * scale


@dataclass(frozen=True)
class BoxSighting:
    """A box in the image, by the pixel where its nearest edge meets the floor."""

    color: str
    column: float
    row: float
    area: int


def _clean(mask: np.ndarray) -> np.ndarray:
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def color_masks(hsv: np.ndarray) -> dict:
    """Return the cleaned-up mask of each box colour."""
    red = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1) | cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
    return {RED: _clean(red), GREEN: _clean(cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER))}


def find_boxes(bgr: np.ndarray) -> List[BoxSighting]:
    """
    Return the boxes whose base can be located in the frame.

    A blob is skipped if it runs off the frame, or if what lies under it is not floor,
    because then its true base is hidden behind something and its position would be
    wrong. A skipped box has either been located earlier or will be located later.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    height, width = hsv.shape[:2]
    sightings = []
    for color, mask in color_masks(hsv).items():
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for label in range(1, count):
            left, top, blob_width, blob_height, area = stats[label]
            bottom = top + blob_height
            if area < MIN_BOX_AREA_PX:
                continue
            if left <= 1 or left + blob_width >= width - 1 or bottom >= height - 2:
                continue
            strip = labels[max(top, bottom - BOTTOM_STRIP_ROWS):bottom] == label
            column = float(np.median(np.nonzero(strip)[1]))
            below = hsv[bottom + 1:bottom + 4, int(column), 1]
            if below.size == 0 or float(below.mean()) > FLOOR_MAX_SATURATION:
                continue
            sightings.append(BoxSighting(color, column, float(bottom), int(area)))
    return sightings


def find_finish_row(bgr: np.ndarray) -> Optional[int]:
    """Return the lowest image row showing the finish zone, None if it is not in view."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = _clean(cv2.inRange(hsv, FINISH_LOWER, FINISH_UPPER))
    rows = np.nonzero(np.count_nonzero(mask, axis=1) >= FINISH_MIN_ROW_COVERAGE * mask.shape[1])[0]
    return int(rows[-1]) if rows.size else None


# ---------------------------------------------------------------------- control


FORWARD = 'FORWARD'
TURN_LEFT = 'TURN_LEFT'
TURN_RIGHT = 'TURN_RIGHT'
FINISHED = 'FINISHED'

# Sightings further than this are too rough to act on, and need no action yet.
MAX_SIGHTING_RANGE = 1.1
# A sighting this close to a remembered box of the same colour is the same box.
SAME_BOX_RADIUS = 0.15
BOX_HALF_DEPTH = 0.025

# Robot centre to box centre, across the track, when passing: half box 0.025 + half
# robot 0.075 + 0.04 of air. A little under box_layout.PASS_CLEARANCE, which layouts
# are generated to allow.
PASS_CLEARANCE = 0.14
# Robot centre to centre line. Wall face 0.5 - half robot 0.075 - 0.035 of air.
LANE_LIMIT = 0.39
# A box starts to constrain the lane this far ahead of the robot's centre, and stops
# once the tail is past it: half robot length 0.095 + half box 0.025 + 0.04 of air.
LOOKAHEAD = 0.70
RELEASE_BEHIND = 0.16
# Base centre past the finish line when stopping: inside the zone, with the nose
# (0.095 ahead) about 3 cm short of the end wall.
STOP_PAST_LINE = 0.07
# Finish line sightings are only trusted within this range.
MAX_FINISH_RANGE = 1.0

LANE_GAIN = 4.0
MAX_HEADING = 0.65
HEADING_GAIN = 3.5
MAX_TURN_RATE = 1.5
ON_LANE_TOLERANCE = 0.015
MIN_SPEED_SCALE = 0.5


@dataclass
class TrackedBox:
    """A box remembered in the odom frame."""

    color: str
    x: float
    y: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Navigator:
    """Remembers what the camera has seen and decides how to steer. No ROS in here."""

    def __init__(self, speed: float = 0.18):
        self.speed = speed
        self.boxes: List[TrackedBox] = []
        self.finish_x: Optional[float] = None
        self.finished = False

    def observe_box(self, color: str, forward: float, left: float,
                    x: float, y: float, yaw: float) -> None:
        """Record a box sighted at (forward, left) of its near edge from pose (x, y, yaw)."""
        if math.hypot(forward, left) > MAX_SIGHTING_RANGE:
            return
        forward += BOX_HALF_DEPTH
        box_x = x + forward * math.cos(yaw) - left * math.sin(yaw)
        box_y = y + forward * math.sin(yaw) + left * math.cos(yaw)
        for box in self.boxes:
            if box.color == color and math.hypot(box.x - box_x, box.y - box_y) < SAME_BOX_RADIUS:
                # Sightings improve as the robot closes in, so the latest counts most.
                box.x += 0.5 * (box_x - box.x)
                box.y += 0.5 * (box_y - box.y)
                return
        self.boxes.append(TrackedBox(color, box_x, box_y))

    def observe_finish(self, forward: float, x: float, yaw: float) -> None:
        """Record the finish line, sighted `forward` of the robot along its heading."""
        if forward <= MAX_FINISH_RANGE:
            self.finish_x = x + forward * math.cos(yaw)

    def allowed_lane(self, x: float) -> Tuple[float, float]:
        """
        Return the y range the boxes in reach allow, nearest box first.

        A red box raises the lowest y allowed (robot to its left) and a green box lowers
        the highest (robot to its right), so red can only ever cause a left turn and
        green a right one. If a further box asks for what a nearer one forbids, the
        nearer one wins and the further one waits its turn.
        """
        low, high = -LANE_LIMIT, LANE_LIMIT
        nearby = [box for box in self.boxes if -RELEASE_BEHIND <= box.x - x <= LOOKAHEAD]
        for box in sorted(nearby, key=lambda item: item.x):
            if box.color == RED:
                needed = min(box.y + PASS_CLEARANCE, LANE_LIMIT)
                if needed > high:
                    break
                low = max(low, needed)
            else:
                needed = max(box.y - PASS_CLEARANCE, -LANE_LIMIT)
                if needed < low:
                    break
                high = min(high, needed)
        return low, high

    def command(self, x: float, y: float, yaw: float) -> Tuple[float, float, str]:
        """Return (linear speed, turn rate, state) for the robot's pose."""
        if self.finished or (self.finish_x is not None and x >= self.finish_x + STOP_PAST_LINE):
            self.finished = True
            return 0.0, 0.0, FINISHED

        low, high = self.allowed_lane(x)
        lane = min(max(y, low), high)
        lane_error = lane - y
        heading = max(-MAX_HEADING, min(MAX_HEADING, LANE_GAIN * lane_error))
        heading_error = wrap_angle(heading - yaw)
        turn = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, HEADING_GAIN * heading_error))
        speed_scale = max(MIN_SPEED_SCALE, 1.0 - abs(heading_error))

        if lane_error > ON_LANE_TOLERANCE:
            state = TURN_LEFT
        elif lane_error < -ON_LANE_TOLERANCE:
            state = TURN_RIGHT
        else:
            state = FORWARD
        return self.speed * speed_scale, turn, state


def main(args=None):
    # Imported here so that the logic above can be tested without a ROS environment.
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    class SmartNavigation(Node):

        def __init__(self):
            super().__init__('smart_navigation')
            self.declare_parameter('linear_speed', 0.18)
            self.navigator = Navigator(self.get_parameter('linear_speed').value)
            self.camera = CameraModel()
            self.bridge = CvBridge()
            self.pose = None
            self.state = None
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.debug_pub = self.create_publisher(Image, '/nav/debug_image', 1)
            # Latched, so that a late subscriber still learns the robot has finished.
            self.state_pub = self.create_publisher(
                String, '/nav/state',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Odometry, '/odom', self.on_odom, 20)
            self.create_subscription(Image, '/camera/image_raw', self.on_image, 1)
            self.get_logger().info('Waiting for the camera and odometry.')

        def on_odom(self, message):
            pose = message.pose.pose
            yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
            self.pose = (pose.position.x, pose.position.y, wrap_angle(yaw))

        def on_image(self, message):
            if self.pose is None:
                return
            x, y, yaw = self.pose
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            sightings = find_boxes(frame)
            for sighting in sightings:
                point = self.camera.floor_point(sighting.column, sighting.row)
                if point is not None:
                    self.navigator.observe_box(sighting.color, point[0], point[1], x, y, yaw)
            finish_row = find_finish_row(frame)
            # At the very bottom of the frame the near edge has gone out of view, so
            # the row no longer marks the line: keep the last estimate.
            if finish_row is not None and finish_row < frame.shape[0] - 3:
                point = self.camera.floor_point(frame.shape[1] / 2, finish_row)
                if point is not None:
                    self.navigator.observe_finish(point[0], x, yaw)

            speed, turn, state = self.navigator.command(x, y, yaw)
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = turn
            self.cmd_pub.publish(twist)
            self.report(state, x, y)
            if self.debug_pub.get_subscription_count() > 0:
                self.publish_debug(frame, sightings, finish_row, state, message.header)

        def report(self, state, x, y):
            if state == self.state:
                return
            self.state = state
            self.state_pub.publish(String(data=state))
            self.get_logger().info(f'{state} at x={x:.2f} y={y:+.2f}, '
                                   f'{len(self.navigator.boxes)} boxes seen')

        def publish_debug(self, frame, sightings, finish_row, state, header):
            shown = frame.copy()
            for sighting in sightings:
                color = (0, 0, 255) if sighting.color == RED else (0, 255, 0)
                cv2.circle(shown, (int(sighting.column), int(sighting.row)), 6, color, 2)
            if finish_row is not None:
                cv2.line(shown, (0, finish_row), (shown.shape[1], finish_row), (255, 0, 255), 2)
            cv2.putText(shown, state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
            debug = self.bridge.cv2_to_imgmsg(shown, encoding='bgr8')
            debug.header = header
            self.debug_pub.publish(debug)

    rclpy.init(args=args)
    node = SmartNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
