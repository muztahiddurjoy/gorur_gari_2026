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
