"""The camera constants live in the xacro and the node; they must not drift apart."""
import os
import re

from fastrobox_sim.box_layout import ROBOT_LENGTH, ROBOT_WIDTH
from fastrobox_sim.smart_navigation import CameraModel
import pytest

XACRO = os.path.join(os.path.dirname(__file__), '..', 'description', 'robot.urdf.xacro')


def prop(name):
    with open(XACRO) as handle:
        return float(re.search(rf'name="{name}" value="([^"]+)"', handle.read()).group(1))


@pytest.mark.parametrize('prop_name, value', [
    ('camera_x', CameraModel.x), ('camera_z', CameraModel.z),
    ('camera_pitch', CameraModel.pitch), ('camera_hfov', CameraModel.hfov),
    ('camera_width', CameraModel.width), ('camera_height', CameraModel.height),
])
def test_node_and_xacro_agree_on_the_camera(prop_name, value):
    assert prop(prop_name) == pytest.approx(value)


def test_layout_knows_the_real_robot_size():
    assert prop('chassis_length') == pytest.approx(ROBOT_LENGTH)
    assert 2 * prop('wheel_y') + prop('wheel_width') == pytest.approx(ROBOT_WIDTH)
