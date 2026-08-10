import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class CustomDisparityExtender(Node):

    def __init__(self):
        super().__init__('disparity_checker_test')

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.lidr_callback, 10)

        # Filled in from the first message; these are only fallbacks.
        # 0.72 deg matches an RPLIDAR C1 at 10 Hz.
        self.min_ang = -math.pi
        self.max_ang = math.pi
        self.ang_inc = math.radians(0.72)

        self.ranges = np.empty(0, dtype=np.float32)
        self.valid = np.empty(0, dtype=bool)

        self.marker_pub = self.create_publisher(MarkerArray, '/tower_markers', 10)

        # Verify against: ros2 topic echo /scan --field header.frame_id
        # C1 drivers publish either 'laser' or 'laser_frame'; a mismatch makes
        # markers vanish silently.
        self.frame_id = 'laser'

        # ── Edge detection ────────────────────────────────────────
        # 0.06 m: a pillar seated 400 mm off the inner wall gives only a small
        # disparity at grazing incidence. 0.10 started missing those past 1.5 m.
        self.spike_threshold = 0.06   # metres, at spike_ref_dist
        self.spike_ref_dist = 0.8     # threshold scales linearly beyond this
        self.edge_stride = 6          # compare ranges[i] against ranges[i-stride]

        # ── Search window ─────────────────────────────────────────
        # Half-angle: the scan is searched over +/- this value.
        self.fov_half_deg = 90.0

        # ── Coarse segment acceptance (pre-tower-gate) ────────────
        # 3.0 cm floor: below that is one ray of noise at working range.
        # 25.0 cm ceiling: the widest legitimate object is the 200 mm parking
        # lot limitation; anything larger is wall.
        self.width_min_cm = 3.0
        self.width_max_cm = 25.0

        # Corridor is 1000 mm wide and the mat 3200 mm. Beyond 2 m a 5 cm
        # pillar subtends <= 2 rays, so the width estimate is meaningless.
        self.max_useful_range = 2.0

        # ── Tower width gate (asymmetric, see module docstring) ───
        self.tower_w_min_face = 5.0    # cm, face-on
        self.tower_w_max_diag = 7.07   # cm, corner-on, 50 * sqrt(2)

        # Pillars sit 200 mm apart centre-to-centre at the tightest. Anything
        # closer than this is the same physical pillar detected twice.
        self.min_tower_separation = 0.10   # metres

    # ──────────────────────────────────────────────────────────────
    # Index <-> angle helpers
    # ──────────────────────────────────────────────────────────────

    def a2i(self, angle: float) -> int:
        """Angle (radians) -> array index. round(), not int(): int() truncates
        toward zero, which is asymmetric either side of 0."""
        return int(round((angle - self.min_ang) / self.ang_inc))

    def i2a(self, index: int) -> float:
        """Array index -> angle (radians)."""
        return self.min_ang + index * self.ang_inc

    # ──────────────────────────────────────────────────────────────
    # Scan handling
    # ──────────────────────────────────────────────────────────────

    def lidr_callback(self, msg: LaserScan):
        # Read geometry from the message every frame. Hardcoding min_ang = -pi
        # breaks a2i() on any LiDAR publishing 0..2pi: the index goes negative
        # and Python silently wraps to the end of the array.
        self.min_ang = msg.angle_min
        self.max_ang = msg.angle_max
        self.ang_inc = msg.angle_increment

        if self.ang_inc <= 0.0:
            self.get_logger().warn('angle_increment is non-positive; dropping scan.')
            return

        r = np.array(msg.ranges, dtype=np.float32)

        # ONE sentinel for every kind of invalid reading. Mapping nan -> inf and
        # inf -> 0 would turn "nothing in range" into "obstacle at 0 m" and
        # manufacture a disparity against the neighbouring ray.
        r = np.nan_to_num(r, nan=0.0, posinf=0.0, neginf=0.0)

        upper = min(float(msg.range_max), self.max_useful_range)
        valid = (r > max(float(msg.range_min), 0.01)) & (r < upper)

        self.ranges = r
        self.valid = valid

        # Gate to tower-sized objects BEFORE deduplication, so a wall fragment
        # cannot occupy the separation radius and suppress a real pillar.
        objects = [o for o in self.find_objects() if self.is_tower(o)]
        objects.sort(key=lambda o: o["dist"])
        towers = self.deduplicate(objects)

        marker_array = MarkerArray()
        for i, o in enumerate(towers):
            marker_array.markers.append(self.make_marker(o, i))

        # Publish unconditionally: an empty array is meaningful information.
        self.marker_pub.publish(marker_array)

        if not towers:
            return

        target = towers[0]
        self.get_logger().info(
            f"{len(towers)} tower(s); closest at {target['dist']:.2f} m, "
            f"width {target['width_cm']:.1f} cm, "
            f"angle {target['angle_deg']:.1f} deg."
        )

    # ──────────────────────────────────────────────────────────────
    # Edge detection + width measurement
    # ──────────────────────────────────────────────────────────────

    def threshold_at(self, dist: float) -> float:
        """Scale the spike threshold with distance.

        At long range the natural range difference between adjacent rays on an
        obliquely-viewed wall approaches a fixed threshold, so the detector
        fires on flat surfaces. Growing the threshold with range keeps it
        sensitive up close without constant false positives far away.
        """
        return self.spike_threshold * max(1.0, dist / self.spike_ref_dist)

    def tower_width_gate(self, dist: float):
        """Return (min_cm, max_cm) accepted for a tower at this distance.

        Geometric band is [5.0, 7.07] cm. Each edge is quantised by up to one
        beam step, and a stride > 1 adds further edge ambiguity, so the band is
        widened by a range-dependent slack.
        """
        ray_cm = dist * self.ang_inc * 100.0
        slack = 1.5 * ray_cm + 0.5 * (self.edge_stride - 1) * ray_cm
        return (self.tower_w_min_face - slack,
                self.tower_w_max_diag + slack)

    def is_tower(self, obj: dict) -> bool:
        lo, hi = self.tower_width_gate(obj["dist"])
        return lo <= obj["width_cm"] <= hi

    def find_objects(self):
        """
        Walk the FOV looking for falling-edge -> rising-edge pairs.

        Falling edge (range drops) = an object begins; push its first ray.
        Rising  edge (range jumps) = the object ends;  pop and measure.

        Using the SIGN is what makes this robust. abs() throws the direction
        away, and pairing elements (0,1),(2,3),... then assumes edges alternate
        perfectly starting with an entering edge. One clipped or missed edge
        shifts every later pair by one and you measure the GAP between two
        objects instead of an object.

        Stride compensation: comparing ranges[i] against ranges[i-stride] means
        the transition lies somewhere in (i-stride, i]. Both edges take the
        earliest possible boundary, so the two biases cancel in the width:
            p = i - stride + 1   (first ray on object)
            q = i - stride       (last ray on object)
        With stride == 1 these reduce to p = i and q = i - 1.
        """
        n = len(self.ranges)
        if n == 0:
            return []

        stride = max(1, self.edge_stride)

        half = math.radians(self.fov_half_deg)
        start = max(stride, self.a2i(-half))
        end = min(n - 1, self.a2i(half))
        if end <= start:
            return []

        stack = []
        objects = []

        for i in range(start, end):
            # Both samples must be real before their difference means anything.
            if not (self.valid[i] and self.valid[i - stride]):
                continue

            near = float(self.ranges[i])
            far = float(self.ranges[i - stride])
            slope = near - far

            thresh = self.threshold_at(min(near, far))

            if slope < -thresh:
                # Range dropped: object starts at the first ray on it.
                stack.append(i - stride + 1)

            elif slope > thresh:
                # Range jumped back out: object ended at the previous ray.
                if not stack:
                    # Rising edge with no matching fall: the object was already
                    # in view when the window opened. Skip rather than pair it
                    # with something unrelated.
                    continue

                p = stack.pop()
                q = i - stride
                if q < p:
                    continue

                obj = self.measure(p, q)
                if obj is not None:
                    objects.append(obj)

        return objects

    def deduplicate(self, objects):
        """
        Merge detections of the same physical tower.

        One tower can register more than once per scan: nested falling edges
        left on the stack, or a bad ray splitting a segment in two. Duplicates
        land within a few cm of each other in Cartesian space, while distinct
        towers are at least min_tower_separation apart. Input is sorted by
        distance, so the nearest (best-measured) detection wins.
        """
        kept = []
        for o in objects:
            a = math.radians(o["angle_deg"])
            x = o["dist"] * math.cos(a)
            y = o["dist"] * math.sin(a)
            if all(math.hypot(x - k["x"], y - k["y"]) >= self.min_tower_separation
                   for k in kept):
                o["x"] = x
                o["y"] = y
                kept.append(o)
        return kept

    def measure(self, p: int, q: int):
        """
        Measure the object occupying rays [p, q] inclusive.

        Distance comes from INSIDE the segment. At a falling edge, ranges[i-1]
        is the background behind the object and ranges[i] is the object itself;
        using the background meant a 5 cm pillar at 1 m in front of a wall at
        3 m measured ~15 cm, off by exactly the ratio of the two.

        Median rather than mean, so one bad ray inside the segment cannot drag
        the estimate.
        """
        seg = self.ranges[p:q + 1]
        seg_valid = self.valid[p:q + 1]
        seg = seg[seg_valid]

        if seg.size == 0:
            return None

        dist = float(np.median(seg))

        # (q - p + 1) rays, each covering one angular increment, is the object's
        # angular footprint. Using (q - p) undercounts by one increment:
        # negligible on a wide object, ~50% on a two-ray one.
        theta = (q - p + 1) * self.ang_inc

        # Arc length s = r * theta. For a body of width a at distance d the
        # subtended angle is ~ a/d, so s ~ a.
        width_cm = dist * theta * 100.0

        if not (self.width_min_cm < width_cm < self.width_max_cm):
            return None

        mid = (p + q) // 2

        return {
            "start": p,
            "end": q,
            "dist": dist,
            "angle_deg": math.degrees(self.i2a(mid)),
            "width_cm": width_cm,
        }

    def make_marker(self, obj: dict, marker_id: int) -> Marker:
        x = obj['x']
        y = obj['y']

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'towers'
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0   # identity: no rotation needed for a flat disk

        # Draw the pillar at its true 5 cm footprint rather than the measured
        # contour width, which inflates toward 7.07 cm when seen corner-on.
        diameter = 0.05
        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = 0.02             # thin: sits like a flat disk on the ground

        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 0.8

        # Markers persist until overwritten or deleted. A short lifetime means
        # a tower that drops out of detection disappears instead of ghosting.
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.3 * 1e9)   # 300 ms

        return m


def main(args=None):
    rclpy.init(args=args)
    node = CustomDisparityExtender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()