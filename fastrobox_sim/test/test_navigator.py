from fastrobox_sim import box_layout
from fastrobox_sim.smart_navigation import (
    FINISHED, FORWARD, GREEN, LANE_LIMIT, MAX_SIGHTING_RANGE, Navigator, PASS_CLEARANCE, RED,
    STOP_PAST_LINE, TURN_LEFT, TURN_RIGHT,
)
import pytest


def navigator_with(color, forward, left):
    navigator = Navigator()
    navigator.observe_box(color, forward, left, 0.0, 0.0, 0.0)
    return navigator


def test_default_is_straight_ahead():
    speed, turn, state = Navigator(speed=0.2).command(0.0, 0.0, 0.0)
    assert speed == pytest.approx(0.2)
    assert turn == 0.0
    assert state == FORWARD


def test_red_box_ahead_turns_left():
    _, turn, state = navigator_with(RED, 0.4, 0.0).command(0.0, 0.0, 0.0)
    assert turn > 0
    assert state == TURN_LEFT


def test_green_box_ahead_turns_right():
    _, turn, state = navigator_with(GREEN, 0.4, 0.0).command(0.0, 0.0, 0.0)
    assert turn < 0
    assert state == TURN_RIGHT


def test_red_box_already_well_to_the_right_needs_no_turn():
    _, turn, state = navigator_with(RED, 0.4, -0.3).command(0.0, 0.0, 0.0)
    assert turn == 0.0
    assert state == FORWARD


def test_red_box_to_the_left_is_still_passed_on_its_left():
    _, turn, state = navigator_with(RED, 0.4, 0.2).command(0.0, 0.0, 0.0)
    assert turn > 0
    assert state == TURN_LEFT


def test_green_box_to_the_right_is_still_passed_on_its_right():
    _, turn, state = navigator_with(GREEN, 0.4, -0.2).command(0.0, 0.0, 0.0)
    assert turn < 0
    assert state == TURN_RIGHT


def test_straightens_up_once_clear_of_the_box():
    navigator = navigator_with(RED, 0.4, 0.0)
    _, turn, state = navigator.command(0.1, PASS_CLEARANCE, 0.0)
    assert turn == pytest.approx(0.0)
    assert state == FORWARD


def test_turned_robot_comes_back_to_the_track_direction():
    _, turn, _ = Navigator().command(1.0, 0.1, 0.5)
    assert turn < 0
    _, turn, _ = Navigator().command(1.0, 0.1, -0.5)
    assert turn > 0


def test_remembered_box_holds_the_lane_until_the_tail_is_past():
    navigator = navigator_with(RED, 0.4, 0.0)
    box_x = navigator.boxes[0].x
    # Alongside and out of the camera's view, still constrained.
    assert navigator.allowed_lane(box_x + 0.05)[0] == pytest.approx(PASS_CLEARANCE)
    # Well past: free again.
    assert navigator.allowed_lane(box_x + 0.3)[0] == -LANE_LIMIT


def test_box_beyond_the_lookahead_does_not_act_yet():
    assert navigator_with(RED, 0.9, 0.0).allowed_lane(0.0)[0] == -LANE_LIMIT


def test_repeat_sightings_merge_and_other_colours_do_not():
    navigator = navigator_with(RED, 0.5, 0.0)
    navigator.observe_box(RED, 0.3, 0.02, 0.2, 0.0, 0.0)
    assert len(navigator.boxes) == 1
    navigator.observe_box(GREEN, 0.5, 0.0, 0.0, 0.0, 0.0)
    assert len(navigator.boxes) == 2


def test_very_far_sightings_are_ignored():
    assert navigator_with(RED, MAX_SIGHTING_RANGE + 0.1, 0.0).boxes == []


def test_sighting_uses_the_robot_pose():
    navigator = navigator_with(RED, 0.3, 0.0)
    assert navigator.boxes[0].x == pytest.approx(0.325)
    navigator = Navigator()
    navigator.observe_box(RED, 0.3, 0.0, 1.0, 0.1, 1.5707963)
    assert navigator.boxes[0].x == pytest.approx(1.0, abs=1e-6)
    assert navigator.boxes[0].y == pytest.approx(0.425, abs=1e-6)


def test_never_steered_into_a_wall():
    assert navigator_with(RED, 0.4, 0.35).allowed_lane(0.0)[0] == LANE_LIMIT


def test_nearer_box_wins_when_two_conflict():
    navigator = Navigator()
    navigator.observe_box(RED, 0.3, 0.2, 0.0, 0.0, 0.0)
    navigator.observe_box(GREEN, 0.6, -0.2, 0.0, 0.0, 0.0)
    low, high = navigator.allowed_lane(0.0)
    assert low == pytest.approx(0.2 + PASS_CLEARANCE)
    assert high == LANE_LIMIT


def test_stops_past_the_finish_line_and_stays_stopped():
    navigator = Navigator()
    navigator.observe_finish(0.5, 2.3, 0.0)
    assert navigator.command(2.8, 0.0, 0.0)[2] != FINISHED
    assert navigator.command(2.8 + STOP_PAST_LINE, 0.0, 0.0) == (0.0, 0.0, FINISHED)
    assert navigator.command(1.0, 0.0, 0.0) == (0.0, 0.0, FINISHED)


def test_far_finish_sighting_is_not_trusted():
    navigator = Navigator()
    navigator.observe_finish(2.0, 0.5, 0.0)
    assert navigator.finish_x is None


def test_layout_clearance_leaves_the_controller_margin():
    # Every generated layout must be drivable at the controller's own limits.
    assert box_layout.PASS_CLEARANCE >= PASS_CLEARANCE
    limit = box_layout.TRACK_HALF_WIDTH - box_layout.ROBOT_WALL_CLEARANCE
    assert limit <= LANE_LIMIT
