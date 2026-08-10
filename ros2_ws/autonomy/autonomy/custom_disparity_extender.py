"""
Disparity edge detector + object width measurement — corrected.

Fixes vs the original:
  1. Object distance is taken from INSIDE the object (median of the segment),
     not from the background ray on the far side of the entering edge.
  2. Edge direction (sign) is used: falling edge pushes, rising edge pops.
     No more blind (0,1),(2,3) pairing.
  3. nan_to_num maps every invalid reading to a single 0.0 sentinel, and a
     validity mask gates every comparison.
  4. Slope is measured over a stride so one physical edge registers once.
  5. angle_min / angle_increment are read from the message every frame.
"""

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
        self.min_ang = -math.pi
        self.max_ang = math.pi
        self.ang_inc = math.radians(0.5)

        self.ranges = np.empty(0, dtype=np.float32)
        self.valid = np.empty(0, dtype=bool)

        self.marker_pub = self.create_publisher(MarkerArray, '/tower_markers', 10)
        self.frame_id = 'laser'   # match your LiDAR's actual TF frame

        # Edge detection
        self.spike_threshold = 0.10   # metres, at the reference distance below
        self.spike_ref_dist = 1.0     # threshold scales linearly beyond this
        self.edge_stride = 2          # compare ranges[i] against ranges[i-stride]

        # Search window
        self.fov_deg = 90.0

        # Accept anything from a 2 cm sliver to a 40 cm crate as an "object"
        self.width_min_cm = 2.0
        self.width_max_cm = 40.0

        # Ignore returns beyond this — noise dominates and the threshold
        # heuristic stops being meaningful.
        self.max_useful_range = 4.0
        self.tower_size_tolerance = 2.0  # cm, for filtering out non-tower objects

        # Real towers are never closer than 150 mm apart, so two detections
        # within this radius must be the same physical tower seen twice.
        self.min_tower_separation = 0.15  # metres

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
        # Read the geometry from the message. Hardcoding min_ang = -pi breaks
        # a2i() on any LiDAR that publishes 0..2pi instead of -pi..pi: the
        # index goes negative and Python silently wraps to the end of the array.
        self.min_ang = msg.angle_min
        self.max_ang = msg.angle_max
        self.ang_inc = msg.angle_increment

        r = np.array(msg.ranges, dtype=np.float32)

        # ONE sentinel for every kind of invalid reading. The original mapped
        # nan -> inf and inf -> 0, which meant "nothing in range" became
        # "obstacle at 0 m" and manufactured a disparity against its neighbour.
        r = np.nan_to_num(r, nan=0.0, posinf=0.0, neginf=0.0)

        upper = min(float(msg.range_max), self.max_useful_range)
        valid = (r > max(float(msg.range_min), 0.01)) & (r < upper)

        self.ranges = r
        self.valid = valid

        objects = self.find_objects()
        objects.sort(key=lambda o: o["dist"])
        objects = self.deduplicate(objects)
        marker_array = MarkerArray()

        if not objects:
            return
        i = 0
        for o in objects:
            if abs(float(o["width_cm"])-5) < self.tower_size_tolerance:
                 marker_array.markers.append(self.make_marker(o, i))
                 i += 1

        self.marker_pub.publish(marker_array)
       # self.get_logger().info(f"Detected {len(objects)} objects, published {i} markers.")
        self.get_logger().info(f"Will approach the closest object at {objects[0]['dist']:.2f} m, width {objects[0]['width_cm']:.1f} cm, angle {objects[0]['angle_deg']:.1f} deg.")
        

    # ──────────────────────────────────────────────────────────────
    # Edge detection + width measurement
    # ──────────────────────────────────────────────────────────────

    def threshold_at(self, dist: float) -> float:
        """Scale the spike threshold with distance.

        At 3 m the natural range difference between adjacent rays on an
        obliquely-viewed wall already approaches 0.1 m, so a fixed threshold
        fires on flat surfaces. Growing it with range keeps the detector
        sensitive up close without going off constantly far away.
        """
        return self.spike_threshold * max(1.0, dist / self.spike_ref_dist)

    def find_objects(self):
        """
        Walk the FOV looking for falling-edge -> rising-edge pairs.

        Falling edge  (range drops) = an object begins; push its NEAR index.
        Rising  edge  (range jumps) = the object ends;  pop and measure.

        Using the SIGN is what makes this robust. abs() throws the direction
        away, and then pairing elements (0,1),(2,3),... assumes edges alternate
        perfectly starting with an entering edge. One clipped or missed edge
        shifts every later pair by one, and you end up measuring the GAP
        between two objects instead of an object.
        """
        n = len(self.ranges)
        if n == 0:
            return []

        stride = self.edge_stride

        # Clamp so neither i-stride nor i+1 can run off either end.
        start = max(stride, self.a2i(-math.radians(self.fov_deg)))
        end = min(n - 1, self.a2i(math.radians(self.fov_deg)))
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
                # Range dropped: object starts here, at the NEAR ray.
                stack.append(i)

            elif slope > thresh:
                # Range jumped back out: object ended at the previous ray.
                if not stack:
                    # Rising edge with no matching fall — the object was
                    # already in view when the window started. Skip it rather
                    # than pair it with something unrelated.
                    continue

                p = stack.pop()
                q = i - 1
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
        left on the stack, or a bad ray splitting a segment in two. All those
        duplicates land within a few cm of each other in Cartesian space,
        while distinct towers are >= min_tower_separation apart. So: keep a
        detection only if it is farther than that from every already-kept one.
        Input is sorted by distance, so the nearest (best-measured) detection
        of each tower wins.
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

        The distance comes from INSIDE the segment. This is the bug that
        dominated the original: at a falling edge, ranges[i] is the background
        behind the object and ranges[i+1] is the object. Recording ranges[i]
        meant r was the wall's distance, so a 5 cm pole at 1 m in front of a
        wall at 3 m measured ~15 cm — off by exactly the ratio of the two.

        Median rather than mean, so one bad ray inside the segment can't drag
        the estimate.
        """
        seg = self.ranges[p:q + 1]
        seg_valid = self.valid[p:q + 1]
        seg = seg[seg_valid]

        if seg.size == 0:
            return None

        dist = float(np.median(seg))

        # (q - p + 1) rays, each covering one angular increment, so this is the
        # object's angular footprint. Using (q - p) would undercount by one
        # increment — negligible on a wide object, ~50% on a 2-ray one.
        theta = (q - p + 1) * self.ang_inc

        # Arc length s = r * theta. For a cylinder of radius a at distance d the
        # subtended angle is 2*asin(a/d) ~= 2a/d, so s ~= 2a = the diameter.
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
        m.pose.orientation.w = 1.0   # identity — no rotation needed for a flat disk

        # Diameter from the measured width, with a floor so it stays visible
        # even for a narrow/noisy detection.
        diameter = max(obj['width_cm'] / 100.0, 0.03)
        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = 0.02             # thin — sits like a flat disk on the ground

        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 0.8

        # Markers persist until overwritten or explicitly deleted. Give it a
        # short lifetime so a tower that drops out of detection actually
        # disappears instead of leaving a ghost marker in RViz.
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