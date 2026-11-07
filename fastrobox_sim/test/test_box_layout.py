import math

from fastrobox_sim.box_layout import (
    Box, BOX_SIDE, footprint_gap, generate_layout, GREEN, is_feasible, MAX_X, MIN_SPACING,
    MIN_X, RED, required_lane, ROBOT_WIDTH, TRACK_HALF_WIDTH,
)
import pytest

SEEDS = range(300)


@pytest.mark.parametrize('count', [6, 7, 8])
def test_generates_the_requested_count(count):
    assert len(generate_layout(count, seed=1)) == count


def test_default_count_is_six_to_eight():
    assert {len(generate_layout(seed=seed)) for seed in SEEDS} == {6, 7, 8}


def test_same_seed_same_layout():
    assert generate_layout(seed=42) == generate_layout(seed=42)
    assert generate_layout(seed=42) != generate_layout(seed=43)


@pytest.mark.parametrize('seed', SEEDS)
def test_layout_respects_the_arena_and_the_rules(seed):
    boxes = generate_layout(seed=seed)
    xs = sorted(box.x for box in boxes)
    assert all(MIN_X - 1e-3 <= x <= MAX_X + 1e-3 for x in xs)
    assert all(b - a >= MIN_SPACING - 2e-3 for a, b in zip(xs, xs[1:]))
    assert {box.color for box in boxes} == {RED, GREEN}
    assert is_feasible(boxes)
    for box in boxes:
        # The legal side has a gap the robot's body fits through.
        low, high = required_lane(box)
        assert high >= low
        if box.color == RED:
            gap = TRACK_HALF_WIDTH - (box.y + BOX_SIDE / 2)
        else:
            gap = (box.y - BOX_SIDE / 2) + TRACK_HALF_WIDTH
        assert gap > ROBOT_WIDTH


def test_red_against_the_left_wall_is_rejected():
    assert not is_feasible([Box(RED, 1.0, 0.42)])
    assert is_feasible([Box(RED, 1.0, -0.42)])


def test_green_against_the_right_wall_is_rejected():
    assert not is_feasible([Box(GREEN, 1.0, -0.42)])
    assert is_feasible([Box(GREEN, 1.0, 0.42)])


def test_lane_change_needs_room_along_the_track():
    too_close = [Box(RED, 1.0, 0.1), Box(GREEN, 1.3, -0.1)]
    far_enough = [Box(RED, 1.0, 0.1), Box(GREEN, 2.4, -0.1)]
    assert not is_feasible(too_close)
    assert is_feasible(far_enough)


def test_too_many_boxes_are_refused():
    with pytest.raises(ValueError):
        generate_layout(9)


def test_footprint_gap():
    box = Box(RED, 1.0, 0.0)
    assert footprint_gap(box, 1.0, 0.0, 0.0) == 0.0
    # 0.15 across: box half 0.025, robot half 0.075, so 0.05 of air.
    assert footprint_gap(box, 1.0, 0.15, 0.0) == pytest.approx(0.05)
    # Straight behind: the robot's half length is 0.095.
    assert footprint_gap(box, 0.8, 0.0, 0.0) == pytest.approx(0.08)
    # Square-on rotated by 90 degrees, the robot's width faces the box.
    assert footprint_gap(box, 0.8, 0.0, math.pi / 2) == pytest.approx(0.1)
