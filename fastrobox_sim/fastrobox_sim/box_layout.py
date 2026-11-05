"""
Random obstacle layouts that a robot can actually complete.

Free space alone is not enough here, because the colour fixes the side: a red box must
be passed on its left and a green box on its right. A red box against the left wall, or
a red box followed closely by a green one further right, would leave a layout with room
to drive but no legal way through. A layout is only returned once it has passed
`is_feasible`, which follows the robot's possible lanes from box to box.

Pure Python with no ROS, so it can be tested on its own. Coordinates are those of
competition_arena.world: +x along the track from 0 to 3 m, y = 0 on the centre line,
+y to the robot's left.
"""
from dataclasses import dataclass
import math
import random
from typing import List, Optional, Sequence, Tuple

RED = 'red'
GREEN = 'green'

BOX_SIDE = 0.05
BOX_HEIGHT = 0.10
TRACK_HALF_WIDTH = 0.5
ROBOT_LENGTH = 0.19
ROBOT_WIDTH = 0.15

# Boxes stay this far past the start zone: the robot needs to see one and react.
MIN_X = 0.45
# And short of the finish line (x = 2.8), so that no box hides it or sits on it.
MAX_X = 2.65
# Centre to centre along the track, so that each box can be dealt with in turn.
MIN_SPACING = 0.30
# No box closer than this to a side wall (box centre to wall face).
WALL_MARGIN = 0.08

# Robot centre to box centre, across the track, while passing it.
PASS_CLEARANCE = 0.15
# The robot's centre stays this far from a side wall face.
ROBOT_WALL_CLEARANCE = 0.11
# Across-track shift the robot is assumed to manage per metre along the track.
LANE_CHANGE_SLOPE = 0.5


@dataclass(frozen=True)
class Box:
    """One obstacle: its colour and the position of its centre on the floor."""

    color: str
    x: float
    y: float


def required_lane(box: Box) -> Tuple[float, float]:
    """Return the range of robot centre y that passes `box` on its legal side."""
    limit = TRACK_HALF_WIDTH - ROBOT_WALL_CLEARANCE
    if box.color == RED:
        return box.y + PASS_CLEARANCE, limit
    return -limit, box.y - PASS_CLEARANCE


def is_feasible(boxes: Sequence[Box]) -> bool:
    """
    Return whether some drivable path passes every box on its legal side.

    Follows the set of lanes the robot could be in, box by box. It must hold its lane
    while its body overlaps a box along the track, so only the distance left over
    between two boxes is available for changing lanes.
    """
    limit = TRACK_HALF_WIDTH - ROBOT_WALL_CLEARANCE
    low, high = -limit, limit
    previous_x = None
    for box in sorted(boxes, key=lambda item: item.x):
        if previous_x is not None:
            free = max(0.0, box.x - previous_x - (ROBOT_LENGTH + BOX_SIDE))
            shift = free * LANE_CHANGE_SLOPE
            low, high = max(-limit, low - shift), min(limit, high + shift)
        lane_low, lane_high = required_lane(box)
        low, high = max(low, lane_low), min(high, lane_high)
        if low > high:
            return False
        previous_x = box.x
    return True


def _sample_positions(count: int, rng: random.Random) -> List[float]:
    """Pick `count` sorted x positions, evenly random and at least MIN_SPACING apart."""
    slack = (MAX_X - MIN_X) - MIN_SPACING * (count - 1)
    if slack < 0:
        raise ValueError(f'{count} boxes do not fit at {MIN_SPACING} m spacing')
    offsets = sorted(rng.uniform(0.0, slack) for _ in range(count))
    return [MIN_X + offset + MIN_SPACING * index for index, offset in enumerate(offsets)]


def generate_layout(count: Optional[int] = None, seed: Optional[int] = None,
                    max_attempts: int = 100000) -> List[Box]:
    """Return a random feasible layout of `count` boxes, by default 6 to 8 of them."""
    rng = random.Random(seed)
    if count is None:
        count = rng.randint(6, 8)
    if not 1 <= count <= 8:
        raise ValueError(f'box count must be between 1 and 8, not {count}')
    y_limit = TRACK_HALF_WIDTH - WALL_MARGIN
    for _ in range(max_attempts):
        boxes = [
            Box(rng.choice((RED, GREEN)), round(x, 3), round(rng.uniform(-y_limit, y_limit), 3))
            for x in _sample_positions(count, rng)
        ]
        # Keep both colours in play; a one-colour layout tests only half of the logic.
        if len({box.color for box in boxes}) == 2 or count == 1:
            if is_feasible(boxes):
                return boxes
    raise RuntimeError(f'no feasible layout of {count} boxes in {max_attempts} attempts')


def box_sdf(name: str, box: Box) -> str:
    """Return the SDF of a solid box standing on the floor."""
    rgb = '1 0 0' if box.color == RED else '0 1 0'
    mass = 0.05
    side_inertia = mass * (BOX_SIDE ** 2 + BOX_HEIGHT ** 2) / 12
    top_inertia = mass * (2 * BOX_SIDE ** 2) / 12
    size = f'{BOX_SIDE} {BOX_SIDE} {BOX_HEIGHT}'
    return (
        '<?xml version="1.0" ?><sdf version="1.8">'
        f'<model name="{name}"><pose>{box.x} {box.y} {BOX_HEIGHT / 2} 0 0 0</pose>'
        '<link name="link"><inertial>'
        f'<mass>{mass}</mass><inertia><ixx>{side_inertia}</ixx><iyy>{side_inertia}</iyy>'
        f'<izz>{top_inertia}</izz></inertia></inertial>'
        f'<collision name="collision"><geometry><box><size>{size}</size></box></geometry>'
        '</collision>'
        f'<visual name="visual"><geometry><box><size>{size}</size></box></geometry>'
        f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
        '<specular>0 0 0 1</specular></material></visual>'
        '</link></model></sdf>'
    )


def footprint_gap(box: Box, x: float, y: float, yaw: float) -> float:
    """Return the distance from a robot footprint at (x, y, yaw) to `box`, 0 if touching."""
    half_box = BOX_SIDE / 2
    half_length, half_width = ROBOT_LENGTH / 2, ROBOT_WIDTH / 2
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    # The box's corners and the robot's, each measured in the other's frame: the
    # separating axes of two rectangles are their own sides.
    dx, dy = box.x - x, box.y - y
    forward = dx * cos_yaw + dy * sin_yaw
    left = -dx * sin_yaw + dy * cos_yaw
    box_reach_robot = half_box * (abs(cos_yaw) + abs(sin_yaw))
    robot_reach_x = half_length * abs(cos_yaw) + half_width * abs(sin_yaw)
    robot_reach_y = half_length * abs(sin_yaw) + half_width * abs(cos_yaw)
    gaps = (
        abs(forward) - half_length - box_reach_robot,
        abs(left) - half_width - box_reach_robot,
        abs(dx) - half_box - robot_reach_x,
        abs(dy) - half_box - robot_reach_y,
    )
    return max(0.0, max(gaps))
