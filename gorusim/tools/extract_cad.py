#!/usr/bin/env python3
"""
extract_cad - turn CAD/WRO_Bot_v2.step into what the simulator consumes.

    gorusim/meshes/<link>__<colour>.stl    visuals, in each link's frame, metres
    gorusim/description/cad_generated.xacro  xacro properties + visual macros
    gorusim/config/cad_geometry.yaml       the same numbers as data, for sim_mcu
                                           and the tests

Everything geometric is measured from the STEP solids (cylinder axes, sphere
centres, bounding boxes), so a change to the car is one re-run of this script.
The outputs are committed, so building and running the simulator does not need
OpenCascade. Regenerate with:

    python3 -m venv /tmp/cadenv
    /tmp/cadenv/bin/pip install cadquery-ocp numpy
    /tmp/cadenv/bin/python gorusim/tools/extract_cad.py

MASSES ARE THE ONE THING THE STEP CANNOT GIVE. Volumes come from the solids;
the densities and component weights in MASS_RULES below are assumptions, marked
as such, because the real car has not been weighed (README: "Weight |
[Placeholder]"). Once it has been, re-run with --total-mass-g <grams> and every
mass and inertia is scaled to match.

Frames. The CAD origin sits on the ground, midway between the axles and on the
centreline, so it is used directly as base_link. CAD has the car driving towards
-Y with +X on its LEFT; REP-103 wants +X forward and +Y left, which is a +90 deg
turn about Z:

    x_robot = -y_cad      y_robot = x_cad      z_robot = z_cad
"""
import argparse
import collections
import os
import re
import struct

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Sphere
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import (XCAFDoc_ColorCurv, XCAFDoc_ColorGen, XCAFDoc_ColorSurf,
                         XCAFDoc_DocumentTool)

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(PKG)

# CAD -> robot rotation (+90 deg about Z), as a matrix acting on column vectors.
R_CAD = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

# Which link a part belongs to: first regex to match the instance path wins,
# anything unmatched is rigidly part of the chassis.
LINK_RULES = [
    (r'frontwheel_Mount\(Mirror\)', 'front_left_knuckle'),
    (r'frontwheel_Mount', 'front_right_knuckle'),
    (r'wheel\(Mirror\)', 'front_left_wheel'),
    (r'Front_wheel_1', 'front_right_wheel'),
    (r'Rear_wheel_left|rear_shaft|Spur Gear 23 teeth \(1\)', 'rear_axle'),
    (r'C1M1', 'laser'),
    (r'LogitechC922', 'camera_link'),
    (r'GY-BNO055', 'imu_link'),
]

# ASSUMPTIONS, not measurements. (regex, 'density', g/cm3) applies to each part;
# (regex, 'group', grams) shares a weight across every part matching it, in
# proportion to volume. First match wins.
MASS_RULES = [
    (r'C1M1', 'group', 110.0),            # RPLIDAR C1 datasheet
    (r'LogitechC922', 'group', 162.0),    # Logitech C920/C922 datasheet
    (r'Servo Motor', 'group', 55.0),      # MG996R datasheet
    (r'JGA25-370', 'group', 85.0),        # JGA25-370 catalogue, typical
    (r'lipo_Battery', 'group', 185.0),    # 105x33x24 mm = typical 3S 2200 mAh pack
    (r'Raspberry Pi', 'group', 46.0),     # Raspberry Pi 4 datasheet
    (r'rear_shaft|Bearing|Ball Bearing|Cage|Ring', 'density', 7.85),  # steel
    (r'Hex Spacer', 'density', 8.5),      # brass standoffs
    (r'Circuit_board', 'density', 1.6),   # populated board average
    (r'wheel', 'density', 1.10),          # rubber tyre on a printed hub
    (r'.*', 'density', 1.00),             # printed parts: PLA 1.24 at ~80% fill
]

# Rotor inertia reflected through the gearbox, added to the rear axle. Typical
# 370-size rotor (2e-6 kg m2) through the ~6.9:1 ratio the tick stream implies.
AXLE_REFLECTED_INERTIA = 2e-6 * 6.9 ** 2

MM = 0.001

_KEEP_ALIVE = []


def label_name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ''


class Part:
    """One leaf solid of the assembly, placed in the world."""

    def __init__(self, path, shape, located, color):
        self.path = path
        self.shape = shape      # as authored, which is what colours are keyed on
        self.located = located  # moved to its world position
        self.color = color


def load_parts(step_path):
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString('XmlOcaf'))
    app.NewDocument(TCollection_ExtendedString('MDTV-XCAF'), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(step_path) != IFSelect_RetDone:
        raise SystemExit(f'cannot read {step_path}')
    reader.Transfer(doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    parts = []

    def own_color(label, shape):
        color = Quantity_Color()
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen, XCAFDoc_ColorCurv):
            if color_tool.GetColor(shape, kind, color):
                return color.Red(), color.Green(), color.Blue()
        return None

    def walk(label, location, path, inherited):
        ref = TDF_Label()
        instance = label_name(label)
        if shape_tool.IsReference_s(label):
            shape_tool.GetReferredShape_s(label, ref)
            location = location.Multiplied(shape_tool.GetLocation_s(label))
            color = own_color(label, shape_tool.GetShape_s(label)) or inherited
            label = ref
        else:
            color = inherited
        name = label_name(label)
        color = own_color(label, shape_tool.GetShape_s(label)) or color
        here = path + [instance or name]
        if shape_tool.IsAssembly_s(label):
            kids = TDF_LabelSequence()
            shape_tool.GetComponents_s(label, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), location, here, color)
            return
        shape = shape_tool.GetShape_s(label)
        if not any(name in p for p in here):
            here[-1] = f'{here[-1]}/{name}'
        parts.append(Part('/'.join(here), shape, shape.Moved(location), color))

    free = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location(), [], None)

    # The colour tool lives inside the document, so the document has to outlive it.
    _KEEP_ALIVE.append((app, doc))
    return parts, color_tool


def first_match(rules, path):
    for rule in rules:
        if re.search(rule[0], path):
            return rule
    raise ValueError(path)


def shape_bbox(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, True, False)
    return np.array(box.Get()[:3]), np.array(box.Get()[3:])


def volume_props(shape):
    """Volume (mm3), centroid (mm) and unit-density inertia at the centroid."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    com = props.CentreOfMass()
    mat = props.MatrixOfInertia()
    inertia = np.array([[mat.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])
    return props.Mass(), np.array([com.X(), com.Y(), com.Z()]), inertia


def faces_of(shape):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        yield TopoDS.Face_s(explorer.Current())
        explorer.Next()


def cylinders(shape):
    """Every cylindrical face as (axis letter, radius, point, face bbox)."""
    found = []
    for face in faces_of(shape):
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() != GeomAbs_Cylinder:
            continue
        cyl = surface.Cylinder()
        axis = cyl.Axis().Direction()
        vec = np.abs([axis.X(), axis.Y(), axis.Z()])
        if vec.max() < 0.99:
            continue
        loc = cyl.Axis().Location()
        found.append(('XYZ'[int(vec.argmax())], cyl.Radius(),
                      np.array([loc.X(), loc.Y(), loc.Z()]), shape_bbox(face)))
    return found


def spheres(shape):
    found = []
    for face in faces_of(shape):
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() == GeomAbs_Sphere:
            sph = surface.Sphere()
            loc = sph.Location()
            found.append((sph.Radius(), np.array([loc.X(), loc.Y(), loc.Z()])))
    return found


def pick(found, axis, radius, tol=0.02):
    hits = [c for c in found if c[0] == axis and abs(c[1] - radius) < tol]
    if not hits:
        raise SystemExit(f'no {axis} cylinder of radius {radius}')
    return hits


def measure_geometry(parts):
    """The numbers that define how the car moves, read off the solids. CAD mm."""
    by = {}
    for part in parts:
        by.setdefault(part.path.split('/')[1] if '/' in part.path else part.path, []).append(part)

    def one(pattern):
        hits = [p for p in parts if re.search(pattern, p.path)]
        if len(hits) != 1:
            raise SystemExit(f'{pattern} matched {len(hits)} parts')
        return hits[0].located

    geo = {}

    # Wheels: the tread is the outermost cylinder of the solid.
    def tyre(pattern):
        shape = one(pattern)
        found = cylinders(shape)
        radius = max(c[1] for c in found if c[0] == 'X')
        faces = [c for c in found if c[0] == 'X' and abs(c[1] - radius) < 1e-3]
        lo = min(c[3][0][0] for c in faces)
        hi = max(c[3][1][0] for c in faces)
        axle = faces[0][2]
        full_lo, full_hi = shape_bbox(shape)
        return {'radius': radius, 'tread_lo_x': lo, 'tread_hi_x': hi, 'y': axle[1], 'z': axle[2],
                'body_lo_x': full_lo[0], 'body_hi_x': full_hi[0]}

    geo['front_left_tyre'] = tyre(r'wheel\(Mirror\)')
    geo['front_right_tyre'] = tyre(r'Front_wheel_1')
    rears = [p.located for p in parts if re.search(r'Rear_wheel_left', p.path)]
    if len(rears) != 2:
        raise SystemExit('expected two rear wheels')
    rear_lefts = []
    for shape in rears:
        found = cylinders(shape)
        radius = max(c[1] for c in found if c[0] == 'X')
        faces = [c for c in found if c[0] == 'X' and abs(c[1] - radius) < 1e-3]
        full_lo, full_hi = shape_bbox(shape)
        rear_lefts.append({'radius': radius,
                           'tread_lo_x': min(c[3][0][0] for c in faces),
                           'tread_hi_x': max(c[3][1][0] for c in faces),
                           'y': faces[0][2][1], 'z': faces[0][2][2],
                           'body_lo_x': full_lo[0], 'body_hi_x': full_hi[0]})
    rear_lefts.sort(key=lambda t: t['tread_lo_x'])
    geo['rear_right_tyre'], geo['rear_left_tyre'] = rear_lefts

    # Kingpins: vertical bores through the front wheel holder.
    bores = pick(cylinders(one(r'front_wheel_holder')), 'Z', 5.0)
    geo['kingpin_x'] = max(abs(c[2][0]) for c in bores)
    geo['kingpin_y'] = float(np.mean([c[2][1] for c in bores]))

    # Tie bar: the two vertical pins, and the horizontal socket for the rod ball.
    tie = cylinders(one(r'steering_p1'))
    pins = pick(tie, 'Z', 4.5)
    geo['tie_pin_x'] = max(abs(c[2][0]) for c in pins)
    geo['tie_pin_y'] = float(np.mean([c[2][1] for c in pins]))
    socket = pick(tie, 'Y', 4.0)[0][2]
    geo['tie_ball_x'] = socket[0]
    geo['tie_ball_z'] = socket[2]

    # Rod: two ball centres.
    balls = sorted({(round(r[1][0], 3), round(r[1][1], 3), round(r[1][2], 3))
                    for r in spheres(one(r'/Rod'))})
    if len(balls) != 2:
        raise SystemExit(f'rod has {len(balls)} ball centres')
    geo['rod_horn_ball'] = list(balls[0])
    geo['rod_tie_ball'] = list(balls[1])

    # Servo output shaft: the horn's boss.
    shaft = pick(cylinders(one(r'servo_horn')), 'Y', 6.0)[0][2]
    geo['servo_shaft_x'] = shaft[0]
    geo['servo_shaft_z'] = shaft[2]

    # LiDAR: axis at the centre of the body, scan plane through the optical
    # window (the eight R3 cylinders around the belt of the housing).
    c1 = one(r'C1M1')
    lo, hi = shape_bbox(c1)
    geo['lidar_xy'] = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]
    geo['lidar_body_z'] = [lo[2], hi[2]]
    geo['lidar_radius'] = (hi[0] - lo[0]) / 2.0
    geo['lidar_scan_z'] = pick(cylinders(c1), 'Y', 3.0)[0][2][2]

    # Camera: the lens glass disc.
    glass = one(r'Logitech_C920Glass')
    lo, hi = shape_bbox(glass)
    geo['camera_lens'] = [(lo[0] + hi[0]) / 2.0, lo[1], (lo[2] + hi[2]) / 2.0]
    pick(cylinders(glass), 'Y', 30.5, tol=0.5)  # axial along Y, so the lens is level

    # Camera body + its arm's head, for the collision box.
    boxes = [shape_bbox(p.located) for p in parts if re.search(r'Logitech_C920:', p.path)]
    geo['camera_body_lo'] = list(np.min([b[0] for b in boxes], axis=0))
    geo['camera_body_hi'] = list(np.max([b[1] for b in boxes], axis=0))

    # IMU: centre of the BNO055 breakout.
    imu_parts = [p.located for p in parts if re.search(r'GY-BNO055', p.path)]
    boxes = [shape_bbox(s) for s in imu_parts]
    lo = np.min([b[0] for b in boxes], axis=0)
    hi = np.max([b[1] for b in boxes], axis=0)
    geo['imu_xyz'] = list((lo + hi) / 2.0)
    return geo


def assign_masses(parts, total_mass_g=None):
    """(part -> grams). Group weights share by volume, the rest use a density."""
    groups = collections.defaultdict(list)
    grams = {}
    report = collections.defaultdict(float)
    for part in parts:
        volume, _, _ = volume_props(part.located)
        volume = abs(volume)
        part.volume = volume
        rule = first_match(MASS_RULES, part.path)
        part.mass_rule = rule[0]
        if rule[1] == 'group':
            groups[rule[0]].append(part)
        else:
            grams[id(part)] = volume * 1e-3 * rule[2]
            report[rule[0]] += grams[id(part)]
    for rule in MASS_RULES:
        if rule[1] != 'group' or rule[0] not in groups:
            continue
        members = groups[rule[0]]
        total_volume = sum(p.volume for p in members) or 1.0
        for part in members:
            grams[id(part)] = rule[2] * part.volume / total_volume
        report[rule[0]] += rule[2]
    total = sum(grams.values())
    if total_mass_g:
        scale = total_mass_g / total
        grams = {k: v * scale for k, v in grams.items()}
        report = {k: v * scale for k, v in report.items()}
    return grams, report


def link_origins(geo):
    """Link frame origins in the robot frame (metres). All frames stay axis aligned."""
    def robot(x, y, z):
        return R_CAD @ (np.array([x, y, z]) * MM)

    fl = geo['front_left_tyre']
    fr = geo['front_right_tyre']
    rl = geo['rear_left_tyre']
    kingpin_z = fl['z']
    return {
        'base_link': np.zeros(3),
        'rear_axle': robot(0.0, rl['y'], rl['z']),
        'front_left_knuckle': robot(geo['kingpin_x'], geo['kingpin_y'], kingpin_z),
        'front_right_knuckle': robot(-geo['kingpin_x'], geo['kingpin_y'], kingpin_z),
        'front_left_wheel': robot((fl['tread_lo_x'] + fl['tread_hi_x']) / 2.0, fl['y'], fl['z']),
        'front_right_wheel': robot((fr['tread_lo_x'] + fr['tread_hi_x']) / 2.0, fr['y'], fr['z']),
        'laser': robot(geo['lidar_xy'][0], geo['lidar_xy'][1], geo['lidar_scan_z']),
        'camera_link': robot(*geo['camera_lens']),
        'imu_link': robot(*geo['imu_xyz']),
    }


def tessellate(part, color_tool, linear, angular):
    """Triangles of one part, CAD mm, split by colour: {rgb: [N x 3 x 3 arrays]}."""
    BRepMesh_IncrementalMesh(part.located, linear, False, angular, True)
    out = collections.defaultdict(list)
    color = Quantity_Color()
    # Colours are keyed on the face as authored, the mesh is on the placed one.
    # Both walks visit the faces in the same order.
    for authored, face in zip(faces_of(part.shape), faces_of(part.located)):
        where = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, where)
        if tri is None:
            continue
        rgb = None
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if color_tool.GetColor(authored, kind, color):
                rgb = (color.Red(), color.Green(), color.Blue())
                break
        rgb = rgb or part.color or (0.6, 0.6, 0.6)
        trsf = where.Transformation()
        nodes = []
        for i in range(1, tri.NbNodes() + 1):
            pnt = tri.Node(i).Transformed(trsf)
            nodes.append((pnt.X(), pnt.Y(), pnt.Z()))
        nodes = np.array(nodes)
        reversed_face = face.Orientation() == TopAbs_REVERSED
        indices = []
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            indices.append((a - 1, c - 1, b - 1) if reversed_face else (a - 1, b - 1, c - 1))
        if indices:
            out[rgb].append(nodes[np.array(indices)])
    return out


def cluster_decimate(triangles, grid):
    """Snap vertices to a grid and drop what collapses.

    Threads, pin headers and SMD parts carry most of the triangles in this file
    and none of the shape the simulator cares about. Snapping moves any surface
    by at most half a grid cell, and every sensor renders these meshes each frame.
    """
    snapped = np.round(triangles / grid) * grid
    a, b, c = snapped[:, 0], snapped[:, 1], snapped[:, 2]
    keep = (np.any(a != b, axis=1) & np.any(b != c, axis=1) & np.any(a != c, axis=1))
    return snapped[keep]


def write_stl(path, triangles):
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    length[length == 0.0] = 1.0
    normals = normals / length
    record = np.zeros(len(triangles), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    record['n'] = normals
    record['v'] = triangles
    with open(path, 'wb') as handle:
        handle.write(b'gorusim mesh, generated from CAD/WRO_Bot_v2.step'.ljust(80, b' '))
        handle.write(struct.pack('<I', len(triangles)))
        handle.write(record.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--step', default=os.path.join(REPO, 'CAD', 'WRO_Bot_v2.step'))
    parser.add_argument('--total-mass-g', type=float, default=None,
                        help='Scale every mass to this measured total weight.')
    parser.add_argument('--linear-deflection', type=float, default=0.4, help='mm')
    parser.add_argument('--angular-deflection', type=float, default=0.7, help='rad')
    parser.add_argument('--grid-mm', type=float, default=0.6,
                        help='Vertex clustering grid applied to the finished meshes.')
    parser.add_argument('--min-part-mm', type=float, default=4.0,
                        help='Parts with a smaller bounding box are left out of the meshes '
                             '(their mass is still counted).')
    args = parser.parse_args()

    parts, color_tool = load_parts(args.step)
    print(f'{len(parts)} solids in {os.path.basename(args.step)}')

    geo = measure_geometry(parts)
    origins = link_origins(geo)
    grams, mass_report = assign_masses(parts, args.total_mass_g)

    # ---- mass properties per link, in the robot frame, about each link origin ----
    links = {name: {'m': 0.0, 'first': np.zeros(3), 'second': np.zeros((3, 3)), 'parts': []}
             for name in origins}
    for part in parts:
        name = first_match(LINK_RULES + [(r'.*', 'base_link')], part.path)[1]
        part.link = name
        links[name]['parts'].append(part)
        mass = grams[id(part)] * 1e-3  # kg
        if mass <= 0.0:
            continue
        volume, com_cad, inertia_cad = volume_props(part.located)
        com = R_CAD @ (com_cad * MM) - origins[name]
        if abs(volume) > 1e-6:
            # OCC gives inertia for unit density in mm5; per unit mass that is mm2.
            inertia = R_CAD @ (inertia_cad / volume * mass * MM ** 2) @ R_CAD.T
        else:
            inertia = np.eye(3) * mass * 1e-6
        link = links[name]
        link['m'] += mass
        link['first'] += mass * com
        # Parallel axis onto the link origin; shifted back to the COM below.
        link['second'] += inertia + mass * (np.dot(com, com) * np.eye(3) - np.outer(com, com))

    for name, link in links.items():
        com = link['first'] / link['m']
        link['com'] = com
        link['inertia'] = link['second'] - link['m'] * (np.dot(com, com) * np.eye(3)
                                                        - np.outer(com, com))
    links['rear_axle']['inertia'][1, 1] += AXLE_REFLECTED_INERTIA

    # ---- meshes ----
    mesh_dir = os.path.join(PKG, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    for stale in os.listdir(mesh_dir):
        if stale.endswith('.stl'):
            os.remove(os.path.join(mesh_dir, stale))
    visuals = collections.defaultdict(dict)
    triangles_total = 0
    for name, link in links.items():
        by_color = collections.defaultdict(list)
        for part in link['parts']:
            lo, hi = shape_bbox(part.located)
            if np.max(hi - lo) < args.min_part_mm:
                continue
            for rgb, tris in tessellate(part, color_tool, args.linear_deflection,
                                        args.angular_deflection).items():
                by_color[tuple(round(c * 8) / 8 for c in rgb)].extend(tris)
        for rgb, tris in by_color.items():
            stacked = cluster_decimate(np.concatenate(tris), args.grid_mm)
            if not len(stacked):
                continue
            robot = stacked.reshape(-1, 3) @ R_CAD.T * MM - origins[name]
            stacked = robot.reshape(-1, 3, 3).astype(np.float32)
            tag = ''.join(f'{int(c * 8):x}' for c in rgb)
            filename = f'{name}__{tag}.stl'
            write_stl(os.path.join(mesh_dir, filename), stacked)
            visuals[name][filename] = rgb
            triangles_total += len(stacked)

    write_yaml(geo, origins, links)
    write_xacro(geo, origins, links, visuals)

    total = sum(link['m'] for link in links.values())
    size = sum(os.path.getsize(os.path.join(mesh_dir, f)) for f in os.listdir(mesh_dir))
    print(f'{triangles_total} triangles, {size / 1e6:.1f} MB across '
          f'{sum(len(v) for v in visuals.values())} meshes')
    print(f'total mass {total * 1000:.0f} g')
    for name, link in links.items():
        print(f'  {name:22s} {link["m"] * 1000:7.1f} g  com {np.round(link["com"] * 1000, 1)} mm')
    print('mass by rule:')
    for rule, value in sorted(mass_report.items(), key=lambda kv: -kv[1]):
        print(f'  {value:7.1f} g  {rule}')


def write_yaml(geo, origins, links):
    def num(value):
        return float(round(float(value), 4))

    fl, fr = geo['front_left_tyre'], geo['front_right_tyre']
    rl, rr = geo['rear_left_tyre'], geo['rear_right_tyre']
    lines = [
        '# GENERATED by gorusim/tools/extract_cad.py from CAD/WRO_Bot_v2.step.',
        '# Do not edit by hand: re-run the tool. All values are CAD millimetres',
        '# (car drives towards -Y, +X is its left), measured from the solids.',
        '',
        f'wheelbase_mm: {num(rl["y"] - fl["y"])}',
        f'front_track_mm: {num((fl["tread_lo_x"] + fl["tread_hi_x"]) / 2 - (fr["tread_lo_x"] + fr["tread_hi_x"]) / 2)}',
        f'rear_track_mm: {num((rl["tread_lo_x"] + rl["tread_hi_x"]) / 2 - (rr["tread_lo_x"] + rr["tread_hi_x"]) / 2)}',
        f'overall_width_mm: {num(max(fl["body_hi_x"], rl["body_hi_x"]) - min(fr["body_lo_x"], rr["body_lo_x"]))}',
        f'front_tyre_radius_mm: {num(fl["radius"])}',
        f'rear_tyre_radius_mm: {num(rl["radius"])}',
        f'front_tread_mm: {num(fl["tread_hi_x"] - fl["tread_lo_x"])}',
        f'rear_tread_mm: {num(rl["tread_hi_x"] - rl["tread_lo_x"])}',
        '',
        '# Steering linkage: servo horn -> rod -> tie bar -> two knuckle arms.',
        f'kingpin_x_mm: {num(geo["kingpin_x"])}',
        f'kingpin_y_mm: {num(geo["kingpin_y"])}',
        f'tie_pin_x_mm: {num(geo["tie_pin_x"])}',
        f'tie_pin_y_mm: {num(geo["tie_pin_y"])}',
        f'tie_ball_x_mm: {num(geo["tie_ball_x"])}',
        f'tie_ball_y_mm: {num(geo["rod_tie_ball"][1])}',
        f'tie_ball_z_mm: {num(geo["tie_ball_z"])}',
        f'horn_ball_x_mm: {num(geo["rod_horn_ball"][0])}',
        f'horn_ball_z_mm: {num(geo["rod_horn_ball"][2])}',
        f'servo_shaft_x_mm: {num(geo["servo_shaft_x"])}',
        f'servo_shaft_z_mm: {num(geo["servo_shaft_z"])}',
        '',
        '# Sensors',
        f'lidar_x_mm: {num(geo["lidar_xy"][0])}',
        f'lidar_y_mm: {num(geo["lidar_xy"][1])}',
        f'lidar_scan_z_mm: {num(geo["lidar_scan_z"])}',
        f'camera_lens_mm: [{num(geo["camera_lens"][0])}, {num(geo["camera_lens"][1])}, {num(geo["camera_lens"][2])}]',
        f'imu_mm: [{num(geo["imu_xyz"][0])}, {num(geo["imu_xyz"][1])}, {num(geo["imu_xyz"][2])}]',
        '',
        f'total_mass_kg: {num(sum(link["m"] for link in links.values()))}',
        '',
    ]
    with open(os.path.join(PKG, 'config', 'cad_geometry.yaml'), 'w') as handle:
        handle.write('\n'.join(lines))


def write_xacro(geo, origins, links, visuals):
    def fmt(value):
        return f'{float(value):.6f}'.rstrip('0').rstrip('.') if abs(value) >= 1e-9 else '0'

    def sci(value):
        return f'{float(value):.6e}'

    fl, rl = geo['front_left_tyre'], geo['rear_left_tyre']
    props = {
        'wheel_radius': fl['radius'] * MM,
        'rear_wheel_radius': rl['radius'] * MM,
        'front_tread': (fl['tread_hi_x'] - fl['tread_lo_x']) * MM,
        'rear_tread': (rl['tread_hi_x'] - rl['tread_lo_x']) * MM,
        'rear_wheel_offset': (rl['tread_lo_x'] + rl['tread_hi_x']) / 2.0 * MM,
    }
    out = [
        '<?xml version="1.0"?>',
        '<!-- GENERATED by gorusim/tools/extract_cad.py from CAD/WRO_Bot_v2.step.',
        '     Do not edit by hand: re-run the tool. -->',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="goruBot">',
    ]
    for name, value in props.items():
        out.append(f'    <xacro:property name="{name}" value="{fmt(value)}" />')
    for name, origin in origins.items():
        out.append(f'    <xacro:property name="{name}_xyz" '
                   f'value="{fmt(origin[0])} {fmt(origin[1])} {fmt(origin[2])}" />')
    for side in ('left', 'right'):
        rel = origins[f'front_{side}_wheel'] - origins[f'front_{side}_knuckle']
        out.append(f'    <xacro:property name="front_{side}_wheel_joint_xyz" '
                   f'value="{fmt(rel[0])} {fmt(rel[1])} {fmt(rel[2])}" />')
    lo, hi = geo['lidar_body_z']
    extras = {
        'lidar_radius': geo['lidar_radius'] * MM,
        'lidar_body_length': (hi - lo) * MM,
        'lidar_body_center_z': ((lo + hi) / 2.0 - geo['lidar_scan_z']) * MM,
    }
    for name, value in extras.items():
        out.append(f'    <xacro:property name="{name}" value="{fmt(value)}" />')
    cam_lo = R_CAD @ (np.array(geo['camera_body_lo']) * MM)
    cam_hi = R_CAD @ (np.array(geo['camera_body_hi']) * MM)
    cam_size = np.abs(cam_hi - cam_lo)
    cam_centre = (cam_lo + cam_hi) / 2.0 - origins['camera_link']
    out.append(f'    <xacro:property name="camera_box_size" '
               f'value="{fmt(cam_size[0])} {fmt(cam_size[1])} {fmt(cam_size[2])}" />')
    out.append(f'    <xacro:property name="camera_box_xyz" '
               f'value="{fmt(cam_centre[0])} {fmt(cam_centre[1])} {fmt(cam_centre[2])}" />')
    for name, link in links.items():
        inertia = link['inertia']
        com = link['com']
        out.append(f'    <xacro:macro name="{name}_cad">')
        for filename, rgb in sorted(visuals[name].items()):
            out.append('        <visual>')
            out.append('            <geometry>')
            out.append(f'                <mesh filename="package://gorusim/meshes/{filename}" />')
            out.append('            </geometry>')
            out.append(f'            <material name="cad_{filename[:-4]}">')
            out.append(f'                <color rgba="{fmt(rgb[0])} {fmt(rgb[1])} {fmt(rgb[2])} 1" />')
            out.append('            </material>')
            out.append('        </visual>')
        out.append('        <inertial>')
        out.append(f'            <origin xyz="{fmt(com[0])} {fmt(com[1])} {fmt(com[2])}" />')
        out.append(f'            <mass value="{fmt(link["m"])}" />')
        out.append(f'            <inertia ixx="{sci(inertia[0, 0])}" ixy="{sci(inertia[0, 1])}" '
                   f'ixz="{sci(inertia[0, 2])}" iyy="{sci(inertia[1, 1])}" '
                   f'iyz="{sci(inertia[1, 2])}" izz="{sci(inertia[2, 2])}" />')
        out.append('        </inertial>')
        out.append('    </xacro:macro>')
    out.append('</robot>')
    out.append('')
    with open(os.path.join(PKG, 'description', 'cad_generated.xacro'), 'w') as handle:
        handle.write('\n'.join(out))


if __name__ == '__main__':
    main()
