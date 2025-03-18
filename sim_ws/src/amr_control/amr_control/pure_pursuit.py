import math


class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(self, dt: float, lookahead_distance: float = 0.5, logger=None):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].

        """
        self._dt: float = dt
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []
        self._logger = logger

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.11. Complete the function body with your code (i.e., compute v and w).
        if not self._path:
            self._logger.info(f"Path not found: {self._path}")
            return 0.0, 0.0

        closest_point, closest_point_idx = self._find_closest_point(x, y)
        destination = self._find_target_point(closest_point, closest_point_idx)

        self._logger.info(f"Current Closest Point: {closest_point}")
        self._logger.info(f"Current Destination: {destination}")

        if destination:
            point = (x, y)
            real_l = self.calculate_dis(point, destination)
            if real_l == 0:
                real_l += 1e-6
            dx = destination[0] - x
            dy = destination[1] - y
            beta = math.atan2(dy, dx)
            alpha = beta - theta
            alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
            e = real_l * math.sin(alpha)

            self._logger.info(f"Alpha: {alpha}")

            # if alpha > math.pi / 4:
            #     v = 0.0
            #     w = -0.2
            # elif alpha < 2 * math.pi - math.pi / 4:
            #     v = 0.0
            #     w = 0.2
            # else:
            #     v = min(2 * e / real_l**2, 0.5)
            #     w = min(2 * v * math.sin(alpha) / real_l, 0.5)

            v = min(2 * e / real_l**2, 0.5)
            w = min(2 * v * math.sin(alpha) / real_l, 0.5)
        else:
            v, w = 0.0, 0.0

        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    def calculate_dis(self, pos_1, pos_2):
        return math.sqrt((pos_1[0] - pos_2[0]) ** 2 + (pos_1[1] - pos_2[1]) ** 2)

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value

    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """

        # TODO: 4.9. Complete the function body (i.e., find closest_xy and closest_idx).
        point = (x, y)
        closest_idx = min(
            range(len(self._path)), key=lambda i: self.calculate_dis(point, self._path[i])
        )

        closest_xy = self._path[closest_idx]

        return closest_xy, closest_idx

    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # TODO: 4.10. Complete the function body with your code (i.e., determine target_xy).
        path_cut = self._path[origin_idx:]

        points_outside_radius = [
            point
            for point in path_cut
            if (self.calculate_dis(origin_xy, point) - self._lookahead_distance) >= 0
        ]

        if points_outside_radius:
            target_xy: tuple[float, float] = points_outside_radius[0]
        else:
            target_xy = self._path[-1]

            if origin_xy == target_xy:
                return None

        return target_xy
