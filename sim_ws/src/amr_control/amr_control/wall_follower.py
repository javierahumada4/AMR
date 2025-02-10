class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        # self.logger_timer = 0

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
        v = 0.15
        w = 0.0
        front = z_scan[0:20] + z_scan[-20:]
        back = z_scan[100:140]
        left = z_scan[20:100]
        right = z_scan[140:220]

        stop_distance = 0.3

        safe_to_move_forward = all(d > stop_distance for d in front if d > 0)
        safe_to_move_backward = all(d > stop_distance for d in back if d > 0)
        safe_to_move_left = all(d > stop_distance for d in left if d > 0)
        safe_to_move_right = all(d > stop_distance for d in right if d > 0)

        if not safe_to_move_forward and z_v > 0:
            v = 0.0
            if safe_to_move_left:
                w = 0.15
            else:
                w = -0.15

        # self.logger_timer += 1
        # if self.logger_timer % 10 == 0:
        #     self.get_logger().info(
        #         f"Safe forward: {safe_to_move_forward}, Safe backward: {safe_to_move_backward}, Safe left: {safe_to_move_left}, Safe right: {safe_to_move_right} \n Published velocities: linear={v}, angular={w}"
        #     )

        return v, w
