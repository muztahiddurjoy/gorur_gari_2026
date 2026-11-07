"""
Place a random set of boxes in the running arena, then exit.

    ros2 run fastrobox_sim spawn_boxes --count 8 --seed 3

Boxes go in through the world's create service, once it is up. The layout is also
written to a YAML file, which is how the referee knows where they are; the robot never
reads it. Replaying a run takes only the seed printed here.
"""
import argparse
import subprocess
import sys
import time

from fastrobox_sim.box_layout import box_sdf, generate_layout
import yaml

DEFAULT_WORLD = 'competition_arena'
DEFAULT_LAYOUT_FILE = '/tmp/fastrobox_layout.yaml'


def _text_format_quote(text):
    """Quote `text` as a protobuf text-format string."""
    return '"' + text.replace('\\', '\\\\').replace('"', '\\"') + '"'


def _wait_for_service(service, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            listed = subprocess.run(['ign', 'service', '--list'], capture_output=True,
                                    text=True, timeout=10)
            if service in listed.stdout.split():
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    return False


def _spawn(service, name, sdf):
    command = [
        'ign', 'service', '-s', service,
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '5000',
        '--req', f'sdf: {_text_format_quote(sdf)}',
    ]
    done = subprocess.run(command, capture_output=True, text=True, timeout=15)
    if done.returncode != 0 or 'data: true' not in done.stdout:
        raise RuntimeError(f'could not spawn {name}: {done.stdout}{done.stderr}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--count', type=int, default=None,
                        help='Number of boxes, 1 to 8. Default: 6 to 8, at random.')
    parser.add_argument('--seed', type=int, default=None,
                        help='Layout seed. Default: a new layout each run.')
    parser.add_argument('--world', default=DEFAULT_WORLD)
    parser.add_argument('--layout-file', default=DEFAULT_LAYOUT_FILE)
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Seconds to wait for the simulator.')
    # ros2 launch appends --ros-args to a Node's argv; this script has no use for them.
    args, _ = parser.parse_known_args(argv)

    seed = args.seed if args.seed is not None else int(time.time())
    boxes = generate_layout(args.count, seed)

    service = f'/world/{args.world}/create'
    if not _wait_for_service(service, args.timeout):
        print(f'spawn_boxes: {service} did not appear; is the simulator running?',
              file=sys.stderr)
        return 1

    records = []
    for index, box in enumerate(boxes):
        name = f'box_{index}_{box.color}'
        _spawn(service, name, box_sdf(name, box))
        records.append({'name': name, 'color': box.color, 'x': box.x, 'y': box.y})
        print(f'spawn_boxes: {name} at x={box.x:.3f} y={box.y:+.3f}', flush=True)

    with open(args.layout_file, 'w') as handle:
        yaml.safe_dump({'seed': seed, 'boxes': records}, handle, sort_keys=False)
    print(f'spawn_boxes: {len(boxes)} boxes, seed {seed}, layout in {args.layout_file}',
          flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
