"""The generated description, sim config and real config have to agree."""
import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import yaml
from gorusim.linkage import LinkageGeometry

PKG = os.path.join(os.path.dirname(__file__), '..')
CONFIG = os.path.join(PKG, 'config')
DESCRIPTION = os.path.join(PKG, 'description')


@pytest.fixture(scope='module')
def geo():
    with open(os.path.join(CONFIG, 'cad_geometry.yaml')) as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope='module')
def sim_mcu_params():
    with open(os.path.join(CONFIG, 'sim_hardware.yaml')) as handle:
        return yaml.safe_load(handle)['sim_mcu']['ros__parameters']


def xacro_properties(filename):
    with open(os.path.join(DESCRIPTION, filename)) as handle:
        return dict(re.findall(r'<xacro:property name="(\w+)" value="([^"]+)"', handle.read()))


def test_the_car_matches_its_spec_sheet(geo):
    # README chassis table: 120 mm wide. The CAD is the source for the rest.
    assert geo['overall_width_mm'] == pytest.approx(120.0, abs=0.01)
    assert geo['wheelbase_mm'] == pytest.approx(140.0, abs=0.01)
    assert geo['rear_tyre_radius_mm'] * 2.0 == pytest.approx(53.0, abs=0.01)


def test_linkage_defaults_match_the_extracted_geometry(geo):
    defaults = LinkageGeometry()
    for name in LinkageGeometry.__dataclass_fields__:
        if name in geo:
            assert getattr(defaults, name) == pytest.approx(geo[name], abs=1e-3), name


def test_steering_stops_agree_everywhere(sim_mcu_params):
    props = xacro_properties('goruBot.xacro')
    for key, param in (('steer_stop_inner', 'inner_stop_deg'), ('steer_stop_outer', 'outer_stop_deg')):
        degrees = float(re.search(r'radians\(([\d.]+)\)', props[key]).group(1))
        assert degrees == sim_mcu_params[param]


def test_motor_speed_agrees(sim_mcu_params):
    props = xacro_properties('control.xacro')
    assert float(props['motor_no_load_speed']) == sim_mcu_params['motor_no_load_speed_rad_s']


def test_encoder_keeps_the_calibration_the_car_runs(geo, sim_mcu_params):
    # vector_odom on the car: 363 ticks, 0.065 m, 0.976. Same distance per tick here.
    real = 0.976 * math.pi * 0.065 / 363
    wheel = math.pi * 2.0 * geo['rear_tyre_radius_mm'] * 0.001
    assert wheel / sim_mcu_params['encoder_counts_per_wheel_rev'] == pytest.approx(real, rel=1e-3)


def test_real_config_is_linked_not_copied():
    for name in ('bot_config.yaml', 'vision_params.yaml', 'disparity_extender_params.yaml'):
        path = os.path.join(CONFIG, name)
        assert os.path.islink(path), f'{name} must link to ros2_ws/config'
        assert os.path.exists(path), f'{name} link is broken'


def test_overlay_only_touches_what_it_must():
    with open(os.path.join(CONFIG, 'sim_overlay.yaml')) as handle:
        overlay = yaml.safe_load(handle)
    keys = {(node, key) for node, body in overlay.items() for key in body['ros__parameters']}
    assert keys == {('mcu_bridge', 'port'),
                    ('open_round_run', 'wait_for_track_ready'),
                    ('disparity_extender_node', 'wait_for_track_ready')}


def test_every_mesh_in_the_description_exists():
    tree = ET.parse(os.path.join(DESCRIPTION, 'cad_generated.xacro'))
    meshes = [m.get('filename') for m in tree.iter('mesh')]
    assert meshes
    for uri in meshes:
        path = os.path.join(PKG, uri.replace('package://gorusim/', ''))
        assert os.path.isfile(path), uri
