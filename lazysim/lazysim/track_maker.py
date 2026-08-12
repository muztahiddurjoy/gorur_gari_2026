"""
track_maker — lays out the traffic pillars, the parking bay and the car's
start pose from config/track.yaml.

Ported from the Gazebo Classic version, which called the ROS services
/spawn_entity and /set_entity_state that libgazebo_ros_factory and
libgazebo_ros_state used to provide. Ignition has no such ROS services;
it exposes its own transport services instead:

    /world/<world>/create     ignition.msgs.EntityFactory -> Boolean
    /world/<world>/set_pose   ignition.msgs.Pose          -> Boolean

Both are served by the UserCommands system, which is why
worlds/lazyWorld.sdf loads it. rclpy cannot speak Ignition transport, so
the requests go out through the `ign service` CLI from ignition-tools.

Pillar colours are not cosmetic. vision_node thresholds red at S=255
exactly (vision_params.yaml red_lower_1 = [0, 255, 87]), so the colours
below keep the off-channels at zero: a red pillar has zero green and
zero blue in both its ambient and diffuse terms, which is what makes it
render as a fully saturated pixel no matter how the scene is lit.
"""
import math
import os
import subprocess

import rclpy
import yaml
from rclpy.node import Node

# ambient, diffuse. Off-channels pinned to zero — see the module docstring.
PILLAR_COLOURS = {
    'red':     ('0.35 0 0 1', '1.0 0 0 1'),
    'green':   ('0 0.35 0 1', '0 0.9 0 1'),
    'magenta': ('0.35 0 0.35 1', '0.9 0 0.9 1'),
    'purple':  ('0.35 0 0.35 1', '0.9 0 0.9 1'),
}
DEFAULT_COLOUR = ('0.3 0.3 0.3 1', '0.7 0.7 0.7 1')

# Pillars and barriers are 100 mm tall, so their centre sits at 50 mm.
# The Classic version dropped them from z=0.5 and let them fall, which
# often landed them tipped over before the run even started.
OBJECT_HALF_HEIGHT = 0.05


def escape_pb_string(text):
    """Escape a Python string for protobuf text format."""
    return (text.replace('\\', '\\\\')
                .replace('"', '\\"')
                .replace('\n', '\\n'))


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TrackMaker(Node):
    def __init__(self):
        super().__init__('track_maker')

        self.declare_parameter('world_name', 'lazyWorld')
        self.declare_parameter('tower_template_path', '')
        self.declare_parameter('wall_template_path', '')
        self.declare_parameter('settings_path', '')
        # How long to keep retrying before giving up on Gazebo coming up.
        self.declare_parameter('service_timeout_sec', 30.0)

        self.world = self.get_parameter('world_name').value
        self.tower_template_path = self.get_parameter('tower_template_path').value
        self.wall_template_path = self.get_parameter('wall_template_path').value
        self.settings_path = self.get_parameter('settings_path').value
        self.service_timeout = float(self.get_parameter('service_timeout_sec').value)

        self.create_service_name = f'/world/{self.world}/create'
        self.set_pose_service_name = f'/world/{self.world}/set_pose'

        self.settings = self.load_yaml(self.settings_path)
        self.tower_template = self.load_text(self.tower_template_path)
        self.wall_template = self.load_text(self.wall_template_path)

    # ══════════════════════════════════════════════════════════════════
    # Loading
    # ══════════════════════════════════════════════════════════════════

    def load_yaml(self, path):
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'Settings file "{path}" does not exist.')
            return None
        with open(path, 'r') as handle:
            return yaml.safe_load(handle)

    def load_text(self, path):
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'Template file "{path}" does not exist.')
            return None
        with open(path, 'r') as handle:
            return handle.read()

    # ══════════════════════════════════════════════════════════════════
    # Ignition transport
    # ══════════════════════════════════════════════════════════════════

    def call_service(self, service, req_type, request, timeout_ms=5000):
        """One `ign service -r` request. Returns True on an OK reply."""
        command = [
            'ign', 'service',
            '-s', service,
            '--reqtype', req_type,
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', str(timeout_ms),
            '--req', request,
        ]
        try:
            done = subprocess.run(command, capture_output=True, text=True,
                                  timeout=timeout_ms / 1000.0 + 5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'{service} timed out.')
            return False
        except FileNotFoundError:
            self.get_logger().error(
                '`ign` not found — install ignition-tools (it ships with '
                'Ignition Fortress) or the track cannot be built.')
            return False

        reply = (done.stdout or '').strip()
        if done.returncode != 0 or 'data: true' not in reply.replace(' ', ' '):
            detail = reply or (done.stderr or '').strip()
            self.get_logger().error(f'{service} failed: {detail}')
            return False
        return True

    def wait_for_gazebo(self):
        """Block until the world's create service shows up."""
        deadline = self.get_clock().now().nanoseconds * 1e-9 + self.service_timeout
        self.get_logger().info(f'Waiting for {self.create_service_name} ...')
        while self.get_clock().now().nanoseconds * 1e-9 < deadline:
            try:
                listed = subprocess.run(['ign', 'service', '--list'],
                                        capture_output=True, text=True, timeout=5.0)
                if self.create_service_name in (listed.stdout or ''):
                    self.get_logger().info('Gazebo is up.')
                    return True
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass
        self.get_logger().error(
            f'{self.create_service_name} never appeared. Is the world named '
            f'"{self.world}", and does it load the UserCommands system?')
        return False

    def spawn(self, name, sdf, x, y, z, yaw=0.0):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        request = (
            f'sdf: "{escape_pb_string(sdf)}" '
            f'name: "{name}" '
            f'allow_renaming: false '
            f'pose: {{ '
            f'position: {{ x: {x} y: {y} z: {z} }} '
            f'orientation: {{ x: {qx} y: {qy} z: {qz} w: {qw} }} '
            f'}}'
        )
        if self.call_service(self.create_service_name,
                             'ignition.msgs.EntityFactory', request):
            self.get_logger().info(f'Spawned {name} at ({x:.2f}, {y:.2f}).')
            return True
        return False

    def set_pose(self, name, x, y, z, yaw_deg):
        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        request = (
            f'name: "{name}" '
            f'position: {{ x: {x} y: {y} z: {z} }} '
            f'orientation: {{ x: {qx} y: {qy} z: {qz} w: {qw} }}'
        )
        if self.call_service(self.set_pose_service_name,
                             'ignition.msgs.Pose', request):
            self.get_logger().info(
                f'Moved {name} to ({x:.2f}, {y:.2f}) facing {yaw_deg:.0f}°.')
            return True
        return False

    # ══════════════════════════════════════════════════════════════════
    # Track layout
    # ══════════════════════════════════════════════════════════════════

    def colours_for(self, name):
        return PILLAR_COLOURS.get(str(name).strip().lower(), DEFAULT_COLOUR)

    def build(self):
        if self.settings is None:
            return
        if not self.wait_for_gazebo():
            return

        if self.settings.get('objects') and self.tower_template:
            for index, tower in enumerate(self.settings.get('towers', []), start=1):
                ambient, diffuse = self.colours_for(tower['color'])
                sdf = self.tower_template.format(
                    name=f'tower_{index}', ambient=ambient, diffuse=diffuse)
                self.spawn(f'tower_{index}', sdf,
                           float(tower['pos']['x']), float(tower['pos']['y']),
                           OBJECT_HALF_HEIGHT)

        self.build_parking(self.settings.get('parking', {}))

        start = self.settings.get('start')
        if start:
            self.place_car(start)

    def build_parking(self, parking):
        if not parking or not parking.get('enabled') or not self.wall_template:
            return

        x = float(parking['pos']['x'])
        y = float(parking['pos']['y'])
        # Bay length plus the 20 mm the barriers themselves take up, so
        # `size` in track.yaml means the clear space between them.
        half = (float(parking['size']) + 0.02) / 2.0
        angle = math.radians(float(parking['angle']))

        ambient, diffuse = self.colours_for('magenta')
        for suffix, sign in (('1', -1.0), ('2', 1.0)):
            name = f'parking_wall_{suffix}'
            sdf = self.wall_template.format(
                name=name, sz_x=0.2, sz_y=0.02, static='false',
                ambient=ambient, diffuse=diffuse)
            self.spawn(name, sdf,
                       x + sign * half * math.cos(angle),
                       y + sign * half * math.sin(angle),
                       OBJECT_HALF_HEIGHT,
                       yaw=angle + math.pi / 2.0)

    def place_car(self, start):
        """Put the car on the start line.

        track.yaml gives where the car's NOSE should sit, so the model
        origin (base_link, the centre of the footprint) is pulled back
        half a car length along the heading.
        """
        yaw_deg = float(start.get('angle', 0.0))
        yaw = math.radians(yaw_deg)
        offset = float(start.get('nose_offset_m', 0.105))  # half of length_m
        self.set_pose(
            start.get('model', 'lazyBot'),
            float(start['pos']['x']) - offset * math.cos(yaw),
            float(start['pos']['y']) - offset * math.sin(yaw),
            0.0,
            yaw_deg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackMaker()
    try:
        node.build()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
