from amr_simulation.robot import Robot
from typing import Any


class TurtleBot3Burger(Robot):
    """Class to control the Turtlebot3 Burger robot."""

    # Constants
    LINEAR_SPEED_MAX = 0.22  # Maximum linear velocity [m/s]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    SENSOR_RANGE_MIN = 0.016  # Minimum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]

    def __init__(self, sim: Any, dt: float) -> None:
        """Turtlebot3 Burger robot class initializer.

        Args:
            sim: CoppeliaSim simulation handle.
            dt: Sampling period [s].

        """
        Robot.__init__(self, sim=sim, track=self.TRACK, wheel_radius=self.WHEEL_RADIUS)
        self._dt: float = dt
        self._motors: dict[str, int] = self._init_motors()

    def move(self, v: float, w: float) -> None:
        """Solve inverse differential kinematics and send commands to the motors.

        If the angular speed of any of the wheels is larger than the maximum admissible,
        sets the larger value to the maximum speed and proportionately scales the other.

        Args:
            v: Linear velocity of the robot center [m/s].
            w: Angular velocity of the robot center [rad/s].

        """
        # TODO: 2.1. Complete the function body with your code (i.e., replace the pass statement).
        left_speed = (v - self.TRACK*w/2)/self.WHEEL_RADIUS
        right_speed = (v + self.TRACK*w/2)/self.WHEEL_RADIUS

        if right_speed > left_speed and right_speed > self.WHEEL_SPEED_MAX:
            left_speed *= self.WHEEL_SPEED_MAX/right_speed
            right_speed = self.WHEEL_SPEED_MAX
        elif left_speed > right_speed and left_speed > self.WHEEL_SPEED_MAX:
            right_speed *= self.WHEEL_SPEED_MAX/left_speed
            left_speed = self.WHEEL_SPEED_MAX

        self._sim.setJointTargetVelocity(self._motors["left"] ,left_speed)
        self._sim.setJointTargetVelocity(self._motors["right"] ,right_speed)

        
    def sense(self) -> tuple[list[float], float, float]:
        """Read the LiDAR and the encoders.

        Returns:
            z_scan: Distance from every LiDAR ray to the closest obstacle in 1.5º increments [m].
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        """
        # Read LiDAR
        packed_data: str = self._sim.getBufferProperty(self._sim.handle_scene, "signal.lidar")
        z_scan: list[float] = self._sim.unpackFloatTable(packed_data)

        # Return nan if the measurement failed
        z_scan = [z if z >= 0.0 else float("nan") for z in z_scan]

        # Read encoders
        z_v, z_w = self._sense_encoders()

        return z_scan, z_v, z_w

    def _init_motors(self) -> dict[str, int]:
        """Acquire motor handles.

        Returns: {'left': handle, 'right': handle}

        """
        motors: dict[str, int] = {}

        motors["left"] = self._sim.getObject("/leftMotor")
        motors["right"] = self._sim.getObject("/rightMotor")

        return motors

    def _sense_encoders(self) -> tuple[float, float]:
        """Solve forward differential kinematics from encoder readings.

        Returns:
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        """
        # Read the angular position increment in the last sampling period [rad]
        encoders: dict[str, float] = {}

        encoders["left"] = self._sim.getFloatProperty(self._sim.handle_scene, "signal.leftEncoder")
        encoders["right"] = self._sim.getFloatProperty(
            self._sim.handle_scene, "signal.rightEncoder"
        )

        # TODO: 2.2. Compute the derivatives of the angular positions to obtain velocities [rad/s].
        left_wheel_velocity = encoders["left"] / self._dt
        right_wheel_velocity = encoders["right"] / self._dt

        # TODO: 2.3. Solve forward differential kinematics (i.e., calculate z_v and z_w).
        z_v = self.WHEEL_RADIUS * (left_wheel_velocity + right_wheel_velocity) / 2
        z_w = self.WHEEL_RADIUS * (right_wheel_velocity - left_wheel_velocity) / (self.TRACK/2)
        
        return z_v, z_w
