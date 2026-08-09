"""
steering_stabilizer.py — Proportional-Derivative (PD) Steering Stabilizer for ROS 2.

Provides shortest-path angle wrapping, deadband filtering, and output clamping
for smooth autonomous steering control across WRO 2026 nodes.
"""

import math


class SteeringStabilizer:
    """
    Modular Heading Controller with Angle Wrapping & Deadband Filtering.
    """

    def __init__(
        self,
        kp: float = 0.03,
        kd: float = 0.005,
        max_steer: float = 1.0,
        tolerance_deg: float = 2.0,
    ) -> None:
        """
        :param kp: Proportional gain (steering effort per degree of heading error)
        :param kd: Derivative gain (damping rate against fast error changes)
        :param max_steer: Maximum allowed steering output magnitude (clamped [-max_steer, +max_steer])
        :param tolerance_deg: Deadband angle threshold in degrees (no steering applied if |error| < tolerance)
        """
        self.kp = kp
        self.kd = kd
        self.max_steer = max_steer
        self.tolerance_deg = tolerance_deg
        self.last_error = 0.0

    def compute_steering(
        self,
        current_heading_deg: float,
        target_heading_deg: float,
        dt: float = 0.025,
    ) -> float:
        """
        Computes normalized steering command [-max_steer, +max_steer] taking
        the shortest angular path from current_heading to target_heading.

        :param current_heading_deg: Actual vehicle heading in degrees.
        :param target_heading_deg:  Target/expected heading in degrees.
        :param dt: Time delta since last update in seconds.
        :return: Steering output value for cmd_vel.angular.z.
        """
        # 1. Shortest path angle wrapping [-180, +180]
        # Prevents full 360-degree spin around the circle when passing 0° / 360° boundary
        error = (target_heading_deg - current_heading_deg + 180.0) % 360.0 - 180.0

        # 2. Deadband filter (suppresses small servo jitter when close to target)
        if abs(error) < self.tolerance_deg:
            self.last_error = 0.0
            return 0.0

        # 3. Derivative calculation
        dt = max(dt, 0.001)
        error_diff = (error - self.last_error) / dt
        self.last_error = error

        # 4. PD Output calculation
        p_term = self.kp * error
        d_term = self.kd * error_diff
        steer_output = -(p_term + d_term)

        # 5. Output clamping
        return max(-self.max_steer, min(self.max_steer, steer_output))
