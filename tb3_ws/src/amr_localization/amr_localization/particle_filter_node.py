import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from amr_msgs.msg import PoseStamped, WallFollowerActive  # Custom message types
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import os
import time
import traceback
from transforms3d.euler import euler2quat  # For converting Euler angles to quaternions

from amr_localization.particle_filter import (
    ParticleFilter,
)  # Custom particle filter implementation
from typing import Optional, Tuple, List


class ParticleFilterNode(LifecycleNode):
    """
    A ROS 2 Lifecycle Node implementing a particle filter for robot localization.

    This node subscribes to odometry and LiDAR scan data, performs particle filtering to estimate
    the robot's pose, and publishes the estimated pose.
    """

    def __init__(self) -> None:
        """Particle filter node initializer."""
        super().__init__("particle_filter")

        # Declare parameters with default values
        self.declare_parameter("dt", 0.05)  # Sampling period for the particle filter
        self.declare_parameter("enable_plot", False)  # Enable visualization
        self.declare_parameter("global_localization", True)  # Global localization
        self.declare_parameter(
            "initial_pose", (0.0, 0.0, math.radians(0))
        )  # Initial pose
        self.declare_parameter(
            "initial_pose_sigma", (0.05, 0.05, math.radians(5))
        )  # Pose uncertainty
        self.declare_parameter("particles", 1000)  # Number of particles
        self.declare_parameter("sigma_v", 0.1)  # Noise for linear velocity
        self.declare_parameter("sigma_w", 0.1)  # Noise for angular velocity
        self.declare_parameter("sigma_z", 0.1)  # Noise for sensor measurements
        self.declare_parameter("steps_btw_sense_updates", 10)  # Steps between updates
        self.declare_parameter("world", "lab03")  # World/map file name

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handles the 'configuring' transition in the node's lifecycle.

        Args:
            state: Current lifecycle state.

        Returns:
            TransitionCallbackReturn: Result of the transition.
        """
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )

        try:
            # Parameters
            dt: float = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot: bool = (
                self.get_parameter("enable_plot").get_parameter_value().bool_value
            )
            global_localization: bool = (
                self.get_parameter("global_localization")
                .get_parameter_value()
                .bool_value
            )
            initial_pose: Tuple[float, float, float] = tuple(
                self.get_parameter("initial_pose")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            initial_pose_sigma: Tuple[float, float, float] = tuple(
                self.get_parameter("initial_pose_sigma")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            particles: int = (
                self.get_parameter("particles").get_parameter_value().integer_value
            )
            sigma_v: float = (
                self.get_parameter("sigma_v").get_parameter_value().double_value
            )
            sigma_w: float = (
                self.get_parameter("sigma_w").get_parameter_value().double_value
            )
            sigma_z: float = (
                self.get_parameter("sigma_z").get_parameter_value().double_value
            )
            world: str = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            self._dt: float = dt  # Sampling period
            self._localized: bool = False  # Localization status
            self._sense_countdown: int = 2  # Timer countdown for sensing
            self._active: bool = False  # Active status of the WallFollower node
            self._movements: List[Tuple[float, float]] = []  # List to store movements
            self._last_lidar: Optional[LaserScan] = None  # Last LiDAR scan data
            self._last_pose: Optional[Tuple[float, float, float]] = (
                None  # Last known pose
            )
            map_path: str = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )  # Path to the map file
            self._particle_filter: ParticleFilter = ParticleFilter(
                dt,
                map_path,
                particle_count=particles,
                sigma_v=sigma_v,
                sigma_w=sigma_w,
                sigma_z=sigma_z,
                global_localization=global_localization,
                initial_pose=initial_pose,
                initial_pose_sigma=initial_pose_sigma,
            )  # Particle filter object

            if self._enable_plot:
                self._particle_filter.show("Initialization", save_figure=True)

            # Create a publisher for pose estimates (PoseStamped messages)
            self._pose_publisher = self.create_publisher(PoseStamped, "/pose", 10)

            # Create a publisher for WallFollowerActive messages
            self._wall_follower_publisher = self.create_publisher(
                WallFollowerActive, "/activateFollower", qos_profile=10
            )

            # Set up QoS profile for LiDAR scan data
            scan_qos_profile: QoSProfile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            # Create subscribers for odometry and LiDAR scan data
            self.encoder_suscriber = self.create_subscription(
                Odometry, "/odometry", callback=self._encoder_callback, qos_profile=10
            )
            self.lidar_subscriber = self.create_subscription(
                LaserScan,
                "/scan",
                callback=self._lidar_callback,
                qos_profile=scan_qos_profile,
            )

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handles the 'activating' transition in the node's lifecycle.

        Args:
            state: Current lifecycle state.

        Returns:
            TransitionCallbackReturn: Result of the transition.
        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")
        msg: WallFollowerActive = (
            WallFollowerActive()
        )  # Create a message to activate the WallFollower node
        msg.active = self._active  # Set the active status
        self._wall_follower_publisher.publish(msg)  # Publish the message
        self.sense_timer = self.create_timer(
            self._sense_countdown, self._timer_callback
        )  # Create a timer for sensing
        return super().on_activate(state)

    def _encoder_callback(self, odom_msg: Odometry) -> None:
        """
        Callback function for odometry messages.
        Stores the movements and updates the robot's pose estimate.
        """
        # Parse odometry data
        z_v: float = odom_msg.twist.twist.linear.x  # Linear velocity
        z_w: float = odom_msg.twist.twist.angular.z  # Angular velocity
        if (
            self._last_pose and self._localized
        ):  # Check if the last pose is available and the robot is localized
            x_old, y_old, theta_old = self._last_pose  # Unpack the last pose

            theta: float = theta_old + z_w * self._dt  # Update the orientation
            theta_mean: float = (
                theta + theta_old
            ) / 2  # Calculate the mean orientation
            x: float = (
                x_old + z_v * math.cos(theta_mean) * self._dt
            )  # Update the x position
            y: float = (
                y_old + z_v * math.sin(theta_mean) * self._dt
            )  # Update the y position

            # Update the last pose and publish the pose estimate
            self._last_pose = (x, y, theta)
            self._publish_pose_estimate(x, y, theta)

        # Check if the robot is localized and the movements are significant
        if abs(z_v) > 0.005 or abs(z_w) > 0.005 and self._active:
            # Store the movements
            self._movements.append((z_v, z_w))

    def _lidar_callback(self, msg: LaserScan) -> None:
        """
        Callback function for LiDAR scan messages.
        Filters the LiDAR data and updates the last LiDAR scan.
        """
        # Check if the LiDAR scan data is valid
        if not msg.ranges:
            self.get_logger().warn("Received empty LiDAR ranges.")
            return

        # Check if the LiDAR scan data is within the valid range
        valid_ranges: List[float] = []
        for r in msg.ranges:
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                valid_ranges.append(float("nan"))
            else:
                valid_ranges.append(r)

        # Create a new LaserScan message with the filtered data
        filtered_msg: LaserScan = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = valid_ranges

        # Update the last LiDAR scan data
        self._last_lidar = filtered_msg

    def _timer_callback(self) -> None:
        """
        Timer callback function for sensing.

        It executes a combined motion step and measurement step of the particle filter.
        """
        # Cancel the timer to prevent multiple calls
        self.sense_timer.cancel()

        # Check if the LiDAR scan data is available
        if self._last_lidar is None:
            self.get_logger().info("No LiDAR measure received.")
            return

        # Stop the WallFollower node by publishing the active status
        self._active = False
        msg: WallFollowerActive = WallFollowerActive()
        msg.active = self._active
        self._wall_follower_publisher.publish(msg)

        # Execute the motion step if there are movements
        if self._movements:
            # Calculate the mean linear and angular velocities
            v_mean: float = sum([z_v for z_v, _ in self._movements]) / len(
                self._movements
            )
            w_mean: float = sum([z_w for _, z_w in self._movements]) / len(
                self._movements
            )

            # Calculate the total time for the motion step
            dt_sum: float = len(self._movements) * self._dt

            # Execute the motion step of the particle filter
            self._execute_motion_step(v_mean, w_mean, dt_sum)

        # Reset the movements list
        self._movements = []

        # Get the LiDAR scan data and execute the measurement step
        z_us: List[float] = self._last_lidar.ranges
        x_h, y_h, theta_h = self._execute_measurement_step(z_us)

        # Publish the pose estimate
        self._publish_pose_estimate(x_h, y_h, theta_h)

        # Update the last pose
        self._last_pose = (x_h, y_h, theta_h)

        # Check if the robot is localized
        if not self._localized:
            # If not localized, set the active status to True and publish the message
            self._active = True
            msg.active = self._active
            self._wall_follower_publisher.publish(msg)

            # Reset the timer for the next sensing step
            self.sense_timer.reset()
        else:
            # If localized, don't reset the timer
            self.get_logger().info(f"The robot is LOCALIZED")

    def _execute_measurement_step(
        self, z_us: List[float]
    ) -> Tuple[float, float, float]:
        """
        Executes and monitors the measurement step (sense) of the particle filter.

        The measurement step updates particle weights based on sensor data and computes the robot's
        pose estimate.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].

        Returns:
            tuple[float, float, float]: Pose estimate (x_h, y_h, theta_h) [m, m, rad].
                                        Returns (inf, inf, inf) if a pose cannot be computed.
        """
        # Default pose when localization fails
        pose: Tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))

        # Resample the particles based on the sensor data
        self._particle_filter.resample(z_us)

        # If the enabled plot is set, show the measurement step
        if self._enable_plot:
            self._particle_filter.show("Sense", save_figure=True)

        # Compute the pose estimate from the resampled particles
        self._localized, pose = self._particle_filter.compute_pose()

        return pose

    def _execute_motion_step(
        self, z_v: float, z_w: float, dt_override: Optional[float] = None
    ) -> None:
        """
        Executes and monitors the motion step (move) of the particle filter.

        The motion step moves particles based on odometric estimates of linear and angular
        velocities.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
            dt_override: Optional override for the sampling period [s].
        """
        # Move the particles based on the odometric estimates
        self._particle_filter.move(z_v, z_w, dt_override)

        # If the enabled plot is set, show the motion step
        if self._enable_plot:
            self._particle_filter.show("Move", save_figure=True)

    def _publish_pose_estimate(self, x_h: float, y_h: float, theta_h: float) -> None:
        """
        Publishes the robot's pose estimate in a custom 'amr_msgs.msg.PoseStamped' message.

        Converts estimated position and orientation into a ROS message format and publishes it.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].
        """
        pose_msg: PoseStamped = PoseStamped()  # Create a PoseStamped message instance

        # Set timestamp in header to current time
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        if self._localized:
            # Fill position fields with estimated coordinates
            pose_msg.pose.position.x = x_h
            pose_msg.pose.position.y = y_h

            # Convert heading angle (theta) to quaternion representation for orientation field
            q: tuple[float, float, float, float] = euler2quat(0.0, 0.0, theta_h)
            pose_msg.pose.orientation.x = q[1]
            pose_msg.pose.orientation.y = q[2]
            pose_msg.pose.orientation.z = q[3]
            pose_msg.pose.orientation.w = q[0]

        # Indicate whether localization was successful in 'localized' field
        pose_msg.localized = self._localized

        # Publish the PoseStamped message to the '/pose' topic
        self._pose_publisher.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    particle_filter_node: ParticleFilterNode = ParticleFilterNode()

    try:
        rclpy.spin(particle_filter_node)
    except KeyboardInterrupt:
        pass

    particle_filter_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
