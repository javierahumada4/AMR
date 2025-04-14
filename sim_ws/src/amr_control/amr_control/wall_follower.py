from typing import List, Tuple


class WallFollower:
    """
    Class to safely explore an environment (without crashing) when the pose is unknown.

    Implements a wall-following algorithm using LiDAR data to navigate while maintaining a safe
    distance from obstacles and walls.
    """

    def __init__(self, dt: float) -> None:
        """
        Initializes the WallFollower class.

        Args:
            dt: Sampling period [s].
        """
        self._dt: float = dt  # Sampling period for control calculations
        self.state: str = "AVANZAR"  # Initial state of the robot ("AVANZAR" or "GIRO")

        # PD controller gains for angular velocity control
        self.Kp: float = 150.0  # Proportional gain
        self.Kd: float = 15.0  # Derivative gain
        self.prev_error: float = 0.0  # Previous error for derivative calculation

        # Distance parameters for obstacle avoidance and wall following
        self.stop_distance: float = 0.2  # Minimum distance to stop before an obstacle [m]
        self.follow_distance: float = 0.2  # Desired distance from the wall [m]

        # Velocity parameters
        self.v: float = 2.0  # Maximum linear velocity [m/s]
        self.w_control: float = 0.2  # Maximum angular velocity for wall following [rad/s]
        self.w_giro: float = 3.0  # Angular velocity for turning [rad/s]

        self.w_actual: float = 0.0  # Current angular velocity during turning

    def compute_commands(self, z_scan: List[float], z_v: float, z_w: float) -> Tuple[float, float]:
        """
        Wall-following exploration algorithm.

        Computes linear and angular velocity commands based on LiDAR data and odometry estimates.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].
        """

        # Select sectors of LiDAR data for analysis
        front: List[float] = list(z_scan[0:20]) + list(z_scan[-20:])  # Front sector (±20 degrees)
        left: List[float] = list(z_scan[40:60])  # Left sector (40-60 degrees)
        right: List[float] = list(z_scan[-60:-40])  # Right sector (-60 to -40 degrees)

        # Filter out invalid (NaN or negative) values from LiDAR data
        valid_front: List[float] = [d for d in front if d > 0.0]
        valid_left: List[float] = [d for d in left if d > 0.0]
        valid_right: List[float] = [d for d in right if d > 0.0]

        # Calculate minimum distances in each sector; default to 0.0 if no valid data exists
        d_front: float = min(valid_front) if valid_front else 0.0
        d_left: float = min(valid_left) if valid_left else 0.0
        d_right: float = min(valid_right) if valid_right else 0.0

        # State: AVANZAR (Move forward while following a wall)
        if self.state == "AVANZAR":
            error: float = (
                self.follow_distance - d_right
            )  # Calculate error relative to desired wall-following distance

            # PD control for angular velocity (w)
            P: float = self.Kp * error  # Proportional term
            D: float = self.Kd * (error - self.prev_error) / self._dt  # Derivative term
            w: float = (
                P + D
            ) * 0.8 + z_w * 0.2  # Smooth control using current angular velocity estimate
            w = (
                min(w, self.w_control) if w > 0 else max(w, -self.w_control)
            )  # Limit angular velocity

            self.prev_error = error  # Update previous error for next iteration

            v: float = min(self.v, z_v + 0.02)  # Gradually accelerate linear velocity

            # Transition to GIRO state if an obstacle is detected in front
            if d_front < self.stop_distance:
                self.state = "GIRO"
                v = 0.0
                w = (
                    self.w_giro if d_left > d_right else -self.w_giro
                )  # Turn direction based on wall proximity
                self.w_actual = w

        # State: GIRO (Turn in place to avoid obstacles)
        elif self.state == "GIRO":
            if (
                d_front > self.stop_distance + 0.15
            ):  # Transition back to AVANZAR when front is clear
                self.state = "AVANZAR"
                v = self.v
                w = -self.w_actual * 0.5  # Smoothly reduce angular velocity after turning
            else:
                v = 0.0
                w = self.w_actual

        return v, w
