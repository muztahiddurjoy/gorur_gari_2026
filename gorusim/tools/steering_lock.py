#!/usr/bin/env python3
"""
steering_lock - where do the tyres hit the chassis?

Rotates each front wheel about its kingpin, using the meshes extract_cad.py wrote,
until it first touches the rest of the car. The result is what
linkage.LinkageGeometry.inner_stop_deg / outer_stop_deg and the kingpin joint limits
in description/goruBotCore.xacro carry.

    pip install trimesh python-fcl numpy
    python3 gorusim/tools/steering_lock.py
"""
import glob
import os

import numpy as np
import trimesh

MESHES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'meshes')
KINGPIN = {'left': (0.070, 0.0297474, 0.0265), 'right': (0.070, -0.0297474, 0.0265)}
WHEEL = {'left': (0.070, 0.0535, 0.0265), 'right': (0.070, -0.0535, 0.0265)}


def load(prefix):
    files = sorted(glob.glob(os.path.join(MESHES, prefix + '__*.stl')))
    return trimesh.util.concatenate([trimesh.load(f) for f in files])


def main():
    chassis = trimesh.collision.CollisionManager()
    chassis.add_object('base_link', load('base_link'))
    for side in ('left', 'right'):
        wheel = load(f'front_{side}_wheel')
        wheel.apply_translation(WHEEL[side])
        for sign, name in ((1, 'left turn'), (-1, 'right turn')):
            for degrees in np.arange(0.0, 91.0, 1.0):
                turn = trimesh.transformations.rotation_matrix(
                    np.radians(sign * degrees), [0, 0, 1], KINGPIN[side])
                moved = wheel.copy()
                moved.apply_transform(turn)
                if chassis.in_collision_single(moved):
                    role = 'inner' if (side == 'left') == (sign > 0) else 'outer'
                    print(f'{side:5s} wheel, {name:10s} ({role}): touches at {degrees:.0f} deg')
                    break


if __name__ == '__main__':
    main()
