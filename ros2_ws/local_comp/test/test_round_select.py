"""config/round_select.yaml: what the start button starts, checked against the firmware."""
import importlib.util
import os
import re

import pytest

WORKSPACE = os.path.join(os.path.dirname(__file__), '..', '..')
ROUND_SELECT = os.path.join(WORKSPACE, 'config', 'round_select.yaml')
FIRMWARE_CONFIG = os.path.join(WORKSPACE, '..', 'firmware', 'include', 'config.h')


def load_launch_file():
    path = os.path.join(WORKSPACE, 'launch', 'gorur_round.launch.py')
    spec = importlib.util.spec_from_file_location('gorur_round_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def launch_file():
    return load_launch_file()


def firmware_constant(name):
    with open(FIRMWARE_CONFIG) as handle:
        return int(re.search(rf'{name} = (\d+)', handle.read()).group(1))


def test_committed_config_is_valid(launch_file):
    round_name, delay = launch_file.load_round_select(ROUND_SELECT)
    assert round_name in launch_file.ROUNDS
    assert delay > 0.0


def test_led_countdown_lasts_exactly_the_start_delay(launch_file):
    # the LED blinks once per START_BLINK_INTERVAL_MS from the press; it must
    # go solid at the moment the car moves
    _, delay = launch_file.load_round_select(ROUND_SELECT)
    led_ms = firmware_constant('START_BLINK_COUNT') * firmware_constant('START_BLINK_INTERVAL_MS')
    assert led_ms == pytest.approx(delay * 1000.0)


def test_both_rounds_map_to_local_comp_executables(launch_file):
    with open(os.path.join(os.path.dirname(__file__), '..', 'setup.py')) as handle:
        setup = handle.read()
    for executable, config in launch_file.ROUNDS.values():
        assert f'{executable} = local_comp.{executable}:main' in setup
        assert os.path.exists(os.path.join(os.path.dirname(__file__), '..', 'config', config))


@pytest.mark.parametrize('text, expected', [
    ('round: open\nstart_delay_sec: 5.0\n', ('open', 5.0)),
    ('round: Obstacle\nstart_delay_sec: 5\n', ('obstacle', 5.0)),
    ('round: " open "\nstart_delay_sec: 2.5\n', ('open', 2.5)),
])
def test_accepts(launch_file, tmp_path, text, expected):
    path = tmp_path / 'round_select.yaml'
    path.write_text(text)
    assert launch_file.load_round_select(str(path)) == expected


@pytest.mark.parametrize('text', [
    'round: parking\nstart_delay_sec: 5.0\n',
    'start_delay_sec: 5.0\n',
    'round: open\n',
    'round: open\nstart_delay_sec: 0\n',
    'round: open\nstart_delay_sec: -1.0\n',
    'round: open\nstart_delay_sec: five\n',
    'round: open\nstart_delay_sec: true\n',
    '- open\n',
    '',
])
def test_rejects(launch_file, tmp_path, text):
    path = tmp_path / 'round_select.yaml'
    path.write_text(text)
    with pytest.raises(RuntimeError):
        launch_file.load_round_select(str(path))
