import math


class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(self, dt: float, lookahead_distance: float = 0.5):
        """
        Initializes the PurePursuit controller.

        Args:
            dt: Sampling period [s], used for time-based control.
            lookahead_distance: Distance to the next target point [m].
        """
        self._dt: float = dt  # Sampling period
        self._lookahead_distance: float = (
            lookahead_distance  # Lookahead distance for target point selection
        )
        self._path: list[tuple[float, float]] = (
            []
        )  # List of (x, y) waypoints representing the path

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """
        Computes linear and angular velocity commands using the pure pursuit algorithm.

        Args:
            x: Current x-coordinate of the robot [m].
            y: Current y-coordinate of the robot [m].
            theta: Current orientation of the robot (heading angle) [rad].

        Returns:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].
        """
        # Check if a path has been provided
        if not self._path:
            return 0.0, 0.0  # Stop the robot if no path is available

        # Find the closest point on the path to the robot's current position
        closest_point, closest_point_idx = self._find_closest_point(x, y)

        # Find the target point based on the lookahead distance
        destination = self._find_target_point(closest_point, closest_point_idx)

        if destination:
            # Calculate the distance and heading to the target point
            point = (x, y)
            real_l = self.calculate_dis(point, destination)
            if real_l == 0:
                real_l += 1e-6  # Avoid division by zero

            dx = destination[0] - x
            dy = destination[1] - y
            beta = math.atan2(
                dy, dx
            )  # Angle to the target point relative to global frame
            alpha = beta - theta  # Angle to the target point relative to robot's frame

            # Normalize alpha to the range [-pi, pi]
            alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

            k_alpha = 1.5  # Gain for angular velocity control

            # Handle sharp turns by zeroing linear velocity and focusing on turning
            if abs(alpha) > math.pi / 4:
                v = 0.0
                w = k_alpha * alpha
            else:
                v = min(
                    2.0, real_l / self._lookahead_distance
                )  # Linear velocity is proportional to distance and capped at 2.0 m/s
                w = (
                    2 * v * math.sin(alpha) / real_l
                )  # Angular velocity based on curvature
        else:
            v, w = 0.0, 0.0  # Stop if no valid target point is found

        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """
        Getter for the path.

        Returns:
            The current path as a list of (x, y) waypoints.
        """
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """
        Setter for the path.

        Args:
            value: A new path represented as a list of (x, y) waypoints.
        """
        self._path = value

    def calculate_dis(
        self, pos_1: tuple[float, float], pos_2: tuple[float, float]
    ) -> float:
        """
        Calculates the Euclidean distance between two points.

        Args:
            pos_1: First point as (x, y).
            pos_2: Second point as (x, y).

        Returns:
            The Euclidean distance between pos_1 and pos_2.
        """
        return math.sqrt((pos_1[0] - pos_2[0]) ** 2 + (pos_1[1] - pos_2[1]) ** 2)

    def _find_closest_point(
        self, x: float, y: float
    ) -> tuple[tuple[float, float], int]:
        """
        Finds the closest point on the path to the robot's current position.

        Args:
            x: Current x-coordinate of the robot [m].
            y: Current y-coordinate of the robot [m].

        Returns:
            A tuple containing:
                - The coordinates of the closest path point as (x, y)."
                - The index of this point in the path.
        """
        point = (x, y)

        # Find index of the closest point by minimizing Euclidean distance
        closest_idx = min(
            range(len(self._path)),
            key=lambda i: self.calculate_dis(point, self._path[i]),
        )

        closest_xy = self._path[closest_idx]  # Coordinates of the closest point

        return closest_xy, closest_idx

    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """
        Finds a target point on the path based on the lookahead distance.

        Args:
            origin_xy: Current position of the robot as (x, y).
            origin_idx: Index of the current closest path point.

        Returns:
            The coordinates of the target point as (x, y), or None if no valid target exists.
        """

        # Slice path from current index onward
        path_cut = self._path[origin_idx:]

        # Find points that are at least lookahead_distance away from origin_xy
        points_outside_radius = [
            point
            for point in path_cut
            if (self.calculate_dis(origin_xy, point) - self._lookahead_distance) >= 0
        ]

        if points_outside_radius:
            target_xy: tuple[float, float] = points_outside_radius[
                0
            ]  # First valid target point
        else:
            target_xy = self._path[
                -1
            ]  # Default to last point in path if no valid targets exist

            if origin_xy == target_xy:  # If already at last waypoint
                return None

        return target_xy
