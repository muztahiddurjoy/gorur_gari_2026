"""The steering linkage, checked against what its geometry has to do."""
import math

import pytest
from gorusim.linkage import (LinkageGeometry, SteeringLinkage, interpolate,
                             turn_radius_m)


@pytest.fixture(scope='module')
def linkage():
    return SteeringLinkage()


def test_cad_pose_is_centre(linkage):
    left, right = linkage.angles(0.0)
    assert left == pytest.approx(0.0, abs=1e-4)
    assert right == pytest.approx(0.0, abs=1e-4)


def test_measured_link_lengths(linkage):
    # Round numbers a designer would pick, recovered from the STEP solids.
    assert linkage.arm == pytest.approx(20.0, abs=0.01)
    assert linkage.horn_radius == pytest.approx(10.0, abs=0.01)
    assert linkage.rod_length == pytest.approx(34.117, abs=0.01)


def test_arms_aim_at_rear_axle_centre():
    geo = LinkageGeometry()
    lean = math.atan2(geo.kingpin_x_mm - geo.tie_pin_x_mm, geo.tie_pin_y_mm - geo.kingpin_y_mm)
    ackermann = math.atan2(geo.kingpin_x_mm, 140.0)
    assert math.degrees(lean) == pytest.approx(math.degrees(ackermann), abs=0.1)


@pytest.mark.parametrize('horn', [10.0, 30.0, 50.0])
def test_positive_horn_steers_left_with_left_wheel_inner(linkage, horn):
    left, right = linkage.angles(math.radians(horn))
    assert left > right > 0.0


@pytest.mark.parametrize('horn', [-10.0, -30.0, -50.0])
def test_negative_horn_steers_right_with_right_wheel_inner(linkage, horn):
    left, right = linkage.angles(math.radians(horn))
    assert right < left < 0.0


def test_close_to_true_ackermann_in_the_working_range(linkage):
    left, right = linkage.angles(math.radians(30.0))
    half_track = 29.7474
    ideal_outer = math.atan2(140.0, 140.0 / math.tan(left) + 2.0 * half_track)
    assert math.degrees(right) == pytest.approx(math.degrees(ideal_outer), abs=1.0)


def test_monotonic_over_full_travel(linkage):
    table = linkage.table(-67.0, 90.0, 1.0)
    for before, after in zip(table, table[1:]):
        assert after[1] >= before[1] - 1e-9
        assert after[2] >= before[2] - 1e-9


def test_tyres_stop_at_the_chassis(linkage):
    left, right = linkage.angles(math.radians(90.0))
    assert math.degrees(right) == pytest.approx(31.0, abs=0.05)
    assert math.degrees(left) <= 39.0


def test_full_right_is_limited_by_the_servo_not_the_chassis(linkage):
    # servoCorrection runs the servo out at -67 deg, before a tyre touches.
    left, right = linkage.angles(math.radians(-67.0))
    assert -math.degrees(right) < 31.0 and -math.degrees(left) < 31.0
    assert left != pytest.approx(right)


def test_interpolate_matches_exact_solution(linkage):
    table = linkage.table(-67.0, 90.0, 0.5)
    for horn in (-40.25, 3.3, 22.7, 58.1):
        exact = linkage.angles(math.radians(horn))
        approx = interpolate(table, math.radians(horn))
        assert approx[0] == pytest.approx(exact[0], abs=1e-3)
        assert approx[1] == pytest.approx(exact[1], abs=1e-3)


def test_interpolate_clamps(linkage):
    table = linkage.table(-67.0, 90.0, 0.5)
    assert interpolate(table, math.radians(-200.0)) == (table[0][1], table[0][2])
    assert interpolate(table, math.radians(200.0)) == (table[-1][1], table[-1][2])


def test_turn_radius_is_infinite_at_centre():
    assert turn_radius_m(0.0, 0.0) == math.inf
    assert turn_radius_m(0.5, 0.4) == pytest.approx(0.140 / math.tan(0.45))
