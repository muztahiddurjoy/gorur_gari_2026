"""
The car's steering linkage, as built.

Nothing on the real car commands a wheel angle. The servo swings a horn, the horn
pushes a rod, the rod shifts a tie bar, and the tie bar turns both knuckles. So
this module does the same: servo angle in, one kingpin angle per wheel out, solved
from the joint positions measured off CAD/WRO_Bot_v2.step
(gorusim/config/cad_geometry.yaml).

Seen from above, in CAD millimetres (the car drives towards -Y, +X is its left):

      K_R ---------------- K_L     kingpins, each knuckle turns about one of these
       \\                  /
        P_R ------------ P_L       tie bar pins, 20 mm behind the kingpins
                  B                ball on the tie bar, carried rigidly with it
                  |
    H (horn) -----+                rod from the horn ball to B, length fixed

The arms lean inward 12 deg, which aims them at the middle of the rear axle: the
textbook Ackermann layout, so the inner wheel turning further than the outer comes
straight out of this geometry and is not assumed anywhere.

Sign convention is REP-103 throughout this module's outputs: a positive kingpin
angle turns the wheel LEFT.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LinkageGeometry:
    """Joint positions, CAD millimetres. Field names match cad_geometry.yaml."""

    kingpin_x_mm: float = 29.7474
    kingpin_y_mm: float = -70.0
    tie_pin_x_mm: float = 25.5906
    tie_pin_y_mm: float = -50.4367
    tie_ball_x_mm: float = 12.0
    tie_ball_y_mm: float = -41.937
    tie_ball_z_mm: float = 31.5
    horn_ball_x_mm: float = -22.117
    horn_ball_z_mm: float = 31.5
    servo_shaft_x_mm: float = -22.256
    servo_shaft_z_mm: float = 21.501
    # Where the tyres hit the chassis, one side, from tools/steering_lock.py.
    # The inner wheel of a turn gets further than the outer before it touches.
    inner_stop_deg: float = 39.0
    outer_stop_deg: float = 31.0


class SteeringLinkage:
    """Maps horn angle -> (left kingpin, right kingpin), both radians, + is left."""

    def __init__(self, geometry=None):
        self.geo = geometry or LinkageGeometry()
        geo = self.geo
        self.arm = math.hypot(geo.tie_pin_x_mm - geo.kingpin_x_mm,
                              geo.tie_pin_y_mm - geo.kingpin_y_mm)
        # Angle of each arm at centre. Left arm points from K_L to P_L.
        self.left_arm_rest = math.atan2(geo.tie_pin_y_mm - geo.kingpin_y_mm,
                                        geo.tie_pin_x_mm - geo.kingpin_x_mm)
        self.right_arm_rest = math.atan2(geo.tie_pin_y_mm - geo.kingpin_y_mm,
                                         -geo.tie_pin_x_mm + geo.kingpin_x_mm)
        self.tie_length = 2.0 * geo.tie_pin_x_mm
        self.horn_radius = math.hypot(geo.horn_ball_x_mm - geo.servo_shaft_x_mm,
                                      geo.horn_ball_z_mm - geo.servo_shaft_z_mm)
        self.horn_rest = math.atan2(geo.horn_ball_x_mm - geo.servo_shaft_x_mm,
                                    geo.horn_ball_z_mm - geo.servo_shaft_z_mm)
        self.rod_length = math.hypot(geo.tie_ball_x_mm - geo.horn_ball_x_mm,
                                     geo.tie_ball_z_mm - geo.horn_ball_z_mm)
        # The ball, expressed in the tie bar's own frame (origin mid-pins).
        self.ball_along = geo.tie_ball_x_mm
        self.ball_ahead = geo.tie_ball_y_mm - geo.tie_pin_y_mm

    # ---- tie bar side -----------------------------------------------------

    def _tie_pose(self, left_angle):
        """Everything the left kingpin angle fixes: right angle, pins, ball."""
        geo = self.geo
        # CAD plan view is mirrored relative to REP-103 yaw only by its axis
        # labels; a turn about +Z is a turn about +Z in both, so + stays left.
        left_pin = (geo.kingpin_x_mm + self.arm * math.cos(self.left_arm_rest + left_angle),
                    geo.kingpin_y_mm + self.arm * math.sin(self.left_arm_rest + left_angle))
        # The right pin is where a circle about K_R (the arm) meets a circle
        # about the left pin (the rigid tie bar). Take the root near rest.
        kx, ky = -geo.kingpin_x_mm, geo.kingpin_y_mm
        dx, dy = left_pin[0] - kx, left_pin[1] - ky
        dist = math.hypot(dx, dy)
        if dist > self.arm + self.tie_length or dist < abs(self.arm - self.tie_length):
            return None
        along = (self.arm ** 2 - self.tie_length ** 2 + dist ** 2) / (2.0 * dist)
        height = math.sqrt(max(self.arm ** 2 - along ** 2, 0.0))
        mx, my = kx + along * dx / dist, ky + along * dy / dist
        candidates = [(mx + height * dy / dist, my - height * dx / dist),
                      (mx - height * dy / dist, my + height * dx / dist)]
        right_pin = max(candidates, key=lambda p: p[1])  # the arms point +Y
        right_angle = (math.atan2(right_pin[1] - ky, right_pin[0] - kx)
                       - self.right_arm_rest)
        # Tie bar frame: unit vector along it, and its normal pointing at the ball.
        ux = (left_pin[0] - right_pin[0]) / self.tie_length
        uy = (left_pin[1] - right_pin[1]) / self.tie_length
        cx = (left_pin[0] + right_pin[0]) / 2.0
        cy = (left_pin[1] + right_pin[1]) / 2.0
        ball = (cx + self.ball_along * ux - self.ball_ahead * uy,
                cy + self.ball_along * uy + self.ball_ahead * ux)
        return right_angle, ball

    def _rod_error(self, left_angle, horn):
        pose = self._tie_pose(left_angle)
        if pose is None:
            return None
        geo = self.geo
        ball = pose[1]
        # Horn angle is measured from the CAD pose, which is servo centre.
        # + horn swings the ball towards -X (the car's right), which pulls the
        # pins that way, and pins behind a kingpin moving right turn a wheel LEFT.
        hx = geo.servo_shaft_x_mm + self.horn_radius * math.sin(self.horn_rest - horn)
        hz = geo.servo_shaft_z_mm + self.horn_radius * math.cos(self.horn_rest - horn)
        # The horn swings in a plane at constant Y, which the CAD pose puts level
        # with the ball, and the tie bar carries the ball off that plane as it moves.
        return (math.hypot(ball[0] - hx, ball[1] - geo.tie_ball_y_mm,
                           geo.tie_ball_z_mm - hz) - self.rod_length)

    def free_angles(self, horn_rad):
        """Kingpin angles for a horn angle, ignoring the chassis stops."""
        # Scan outward for a sign change rather than assuming a bracket: past
        # about 40 deg the arm and tie bar can no longer close at all, so the far
        # ends of any fixed bracket have no solution to bisect against.
        low = high = None
        err_low = None
        previous = None
        for tenth in range(-700, 701, 10):
            angle = math.radians(tenth / 10.0)
            error = self._rod_error(angle, horn_rad)
            if previous is not None and error is not None and previous[1] * error <= 0.0:
                low, err_low, high = previous[0], previous[1], angle
                break
            previous = None if error is None else (angle, error)
        if low is None:
            raise ValueError(f'no linkage solution at horn {math.degrees(horn_rad):.1f} deg')
        for _ in range(60):
            mid = (low + high) / 2.0
            err_mid = self._rod_error(mid, horn_rad)
            if err_mid is None:
                raise ValueError(f'tie bar cannot close at {math.degrees(mid):.1f} deg')
            if err_low * err_mid <= 0.0:
                high = mid
            else:
                low, err_low = mid, err_mid
        left = (low + high) / 2.0
        right = self._tie_pose(left)[0]
        return left, right

    def angles(self, horn_rad):
        """Kingpin angles with the tyres stopped by the chassis.

        There is one degree of freedom, so when either tyre touches, the whole
        linkage stops there and the servo just stalls against it.
        """
        if self._reachable(horn_rad):
            return self.free_angles(horn_rad)
        # Walk back to the horn angle where the first tyre just touches. Far
        # enough past that point the arm and tie bar cannot close at all, so
        # "no solution" counts as past the stop too.
        low, high = 0.0, horn_rad
        for _ in range(40):
            mid = (low + high) / 2.0
            if self._reachable(mid):
                low = mid
            else:
                high = mid
        return self.free_angles(low)

    def _reachable(self, horn_rad):
        try:
            return self._within_stops(*self.free_angles(horn_rad))
        except ValueError:
            return False

    def _within_stops(self, left, right):
        inner = math.radians(self.geo.inner_stop_deg)
        outer = math.radians(self.geo.outer_stop_deg)
        if left >= 0.0 and right >= 0.0:      # turning left: left wheel is inner
            return left <= inner and right <= outer
        if left <= 0.0 and right <= 0.0:      # turning right: right wheel is inner
            return -right <= inner and -left <= outer
        return True                           # straddling centre, nowhere near

    def table(self, low_deg, high_deg, step_deg=0.5):
        """Precomputed [(horn_rad, left_rad, right_rad)], for fast interpolation."""
        rows = []
        count = int(round((high_deg - low_deg) / step_deg))
        for i in range(count + 1):
            horn = math.radians(low_deg + i * step_deg)
            rows.append((horn,) + self.angles(horn))
        return rows


def interpolate(table, horn_rad):
    """Linear lookup in a table() result. Clamps at the ends."""
    if horn_rad <= table[0][0]:
        return table[0][1], table[0][2]
    if horn_rad >= table[-1][0]:
        return table[-1][1], table[-1][2]
    step = table[1][0] - table[0][0]
    i = min(int((horn_rad - table[0][0]) / step), len(table) - 2)
    t = (horn_rad - table[i][0]) / step
    return (table[i][1] + t * (table[i + 1][1] - table[i][1]),
            table[i][2] + t * (table[i + 1][2] - table[i][2]))


def turn_radius_m(left_rad, right_rad, wheelbase_m=0.140):
    """Rear-axle centreline radius implied by the mean steer angle. inf at centre."""
    mean = (left_rad + right_rad) / 2.0
    if abs(mean) < 1e-6:
        return math.inf
    return wheelbase_m / math.tan(abs(mean))
