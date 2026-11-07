import math

import cv2
from fastrobox_sim.smart_navigation import (
    CameraModel, find_boxes, find_finish_row, GREEN, RED,
)
import numpy as np
import pytest

CAMERA = CameraModel()
FLOOR = (208, 208, 208)
SKY = (235, 230, 225)
# BGR, as these render in competition_arena: box faces and the finish checks.
BOX_RED = (0, 0, 140)
BOX_GREEN = (0, 150, 0)
CHECK_GREEN = (68, 108, 42)
START_GREEN = (178, 245, 178)


def blank():
    image = np.full((CAMERA.height, CAMERA.width, 3), FLOOR, np.uint8)
    image[:90] = SKY
    return image


def test_centre_of_frame_meets_the_floor_where_the_tilt_says():
    forward, left = CAMERA.floor_point(320, 240)
    assert forward == pytest.approx(CAMERA.x + CAMERA.z / math.tan(CAMERA.pitch), abs=1e-6)
    assert left == pytest.approx(0.0)


def test_left_of_the_image_is_left_of_the_robot():
    assert CAMERA.floor_point(100, 400)[1] > 0
    assert CAMERA.floor_point(500, 400)[1] < 0


def test_nearer_floor_is_lower_in_the_image():
    assert CAMERA.floor_point(320, 450)[0] < CAMERA.floor_point(320, 300)[0]


def test_nothing_on_the_floor_at_or_above_the_horizon():
    horizon = 240 - CAMERA.focal * math.tan(CAMERA.pitch)
    assert CAMERA.floor_point(320, horizon - 5) is None


@pytest.mark.parametrize('color, paint', [(RED, BOX_RED), (GREEN, BOX_GREEN)])
def test_finds_a_solid_box_by_its_base(color, paint):
    image = blank()
    cv2.rectangle(image, (300, 200), (340, 300), paint, -1)
    sightings = find_boxes(image)
    assert len(sightings) == 1
    assert sightings[0].color == color
    assert sightings[0].column == pytest.approx(320, abs=1)
    assert sightings[0].row == pytest.approx(301, abs=1)


def test_finds_both_colours_in_one_frame():
    image = blank()
    cv2.rectangle(image, (100, 200), (130, 260), BOX_RED, -1)
    cv2.rectangle(image, (400, 200), (430, 260), BOX_GREEN, -1)
    assert {sighting.color for sighting in find_boxes(image)} == {RED, GREEN}


def test_finish_checks_and_start_zone_are_not_boxes():
    image = blank()
    image[150:200, 0:320] = CHECK_GREEN
    image[300:400] = START_GREEN
    assert find_boxes(image) == []


def test_noise_specks_are_not_boxes():
    image = blank()
    cv2.rectangle(image, (300, 300), (303, 303), BOX_RED, -1)
    assert find_boxes(image) == []


def test_box_cut_off_by_the_frame_is_not_located():
    image = blank()
    cv2.rectangle(image, (0, 200), (40, 300), BOX_RED, -1)
    cv2.rectangle(image, (300, 400), (340, 479), BOX_GREEN, -1)
    assert find_boxes(image) == []


def test_box_standing_behind_another_is_not_located():
    image = blank()
    cv2.rectangle(image, (300, 150), (340, 220), BOX_GREEN, -1)
    cv2.rectangle(image, (280, 221), (360, 320), BOX_RED, -1)
    assert [sighting.color for sighting in find_boxes(image)] == [RED]


def test_same_colour_boxes_merged_in_view_locate_the_nearest():
    image = blank()
    cv2.rectangle(image, (325, 150), (360, 240), BOX_RED, -1)
    cv2.rectangle(image, (300, 230), (340, 320), BOX_RED, -1)
    sightings = find_boxes(image)
    assert len(sightings) == 1
    assert sightings[0].column == pytest.approx(320, abs=1)


def test_finish_row_is_the_near_edge_of_the_checks():
    image = blank()
    for start in range(0, 640, 120):
        image[300:340, start:start + 60] = CHECK_GREEN
    assert find_finish_row(image) == pytest.approx(339, abs=1)


def test_no_finish_in_a_bare_frame_or_a_green_box():
    image = blank()
    assert find_finish_row(image) is None
    cv2.rectangle(image, (200, 100), (440, 400), BOX_GREEN, -1)
    assert find_finish_row(image) is None
