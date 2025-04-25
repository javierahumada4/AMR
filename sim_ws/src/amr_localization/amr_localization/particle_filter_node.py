import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

import message_filters
from amr_msgs.msg import PoseStamped  # Custom message type for pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import os
import traceback
from transforms3d.euler import euler2quat  # For converting Euler angles to quaternions

from amr_localization.particle_filter import (
    ParticleFilter,
)  # Custom particle filter implementation


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
        self.declare_parameter(
            "enable_plot", False
        )  # Whether to enable visualization of the particle filter
        self.declare_parameter(
            "global_localization", True
        )  # Whether to use global localization
        self.declare_parameter(
            "initial_pose", (0.0, 0.0, math.radians(0))
        )  # Initial pose (x, y, theta)
        self.declare_parameter(
            "initial_pose_sigma", (0.05, 0.05, math.radians(5))
        )  # Uncertainty in initial pose
        self.declare_parameter("particles", 1000)  # Number of particles in the filter
        self.declare_parameter(
            "sigma_v", 0.1
        )  # Noise standard deviation for linear velocity
        self.declare_parameter(
            "sigma_w", 0.1
        )  # Noise standard deviation for angular velocity
        self.declare_parameter(
            "sigma_z", 0.1
        )  # Noise standard deviation for sensor measurements
        self.declare_parameter(
            "steps_btw_sense_updates", 10
        )  # Steps between sensor updates
        self.declare_parameter("world", "lab03")  # Name of the world/map file

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handles the 'configuring' transition in the node's lifecycle.

        Initializes parameters, sets up subscriptions and publishers, and prepares the particle
        filter.

        Args:
            state: Current lifecycle state.

        Returns:
            TransitionCallbackReturn: Result of the transition.
        """
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )

        try:
            # Retrieve parameters
            dt: float = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot: bool = (
                self.get_parameter("enable_plot").get_parameter_value().bool_value
            )
            global_localization: bool = (
                self.get_parameter("global_localization")
                .get_parameter_value()
                .bool_value
            )
            initial_pose: tuple[float, float, float] = tuple(
                self.get_parameter("initial_pose")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            initial_pose_sigma: tuple[float, float, float] = tuple(
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
            self._steps_btw_sense_updates: int = (
                self.get_parameter("steps_btw_sense_updates")
                .get_parameter_value()
                .integer_value
            )
            world: str = self.get_parameter("world").get_parameter_value().string_value

            # Initialize attributes and particle filter object
            self._localized: bool = False  # Indicates whether the robot is localized
            self._steps: int = 0  # Step counter for sensor updates

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

            # Set up QoS profile for LiDAR scan data
            scan_qos_profile: QoSProfile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            # Create a list of subscribers for odometry and LiDAR scan data
            self._subscribers: list[message_filters.Subscriber] = []
            self._subscribers.append(
                message_filters.Subscriber(self, Odometry, "odometry")
            )
            self._subscribers.append(
                message_filters.Subscriber(
                    self, LaserScan, "scan", qos_profile=scan_qos_profile
                )
            )

            # Create an approximate time synchronizer to synchronize the subscribers
            ts: message_filters.ApproximateTimeSynchronizer = (
                message_filters.ApproximateTimeSynchronizer(
                    self._subscribers, queue_size=10, slop=9
                )
            )
            ts.registerCallback(self._compute_pose_callback)

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
        return super().on_activate(state)

    def _compute_pose_callback(self, odom_msg: Odometry, scan_msg: LaserScan) -> None:
        """
        Callback function for synchronized subscribers.

        Executes motion and measurement steps of the particle filter using odometry and LiDAR data.
        Publishes computed pose estimates.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR sensor readings.
        """
        # Parse measurements from odometry and LiDAR messages
        z_v: float = (
            odom_msg.twist.twist.linear.x
        )  # Linear velocity from odometry [m/s]
        z_w: float = (
            odom_msg.twist.twist.angular.z
        )  # Angular velocity from odometry [rad/s]
        z_scan: list[float] = (
            scan_msg.ranges
        )  # LiDAR scan ranges (distances to obstacles) [m]

        # Execute the motion step of the particle filter
        self._execute_motion_step(z_v, z_w)

        # Execute the measurement step of the particle filter and get pose estimates
        x_h, y_h, theta_h = self._execute_measurement_step(z_scan)

        # Increment step counter for sensor updates
        self._steps += 1

        # Publish the estimated pose
        self._publish_pose_estimate(x_h, y_h, theta_h)

    def _execute_measurement_step(
        self, z_us: list[float]
    ) -> tuple[float, float, float]:
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
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))

        # Perform sense step if localized or if enough steps have passed since last update
        if self._localized or not self._steps % self._steps_btw_sense_updates:
            # Update particle weights based on sensor measurements
            self._particle_filter.resample(z_us)

            # Visualize sense step if enabled
            if self._enable_plot:
                self._particle_filter.show("Sense", save_figure=True)

            # Compute robot's estimated pose using particle clustering
            self._localized, pose = self._particle_filter.compute_pose()

        return pose

    def _execute_motion_step(self, z_v: float, z_w: float) -> None:
        """
        Executes and monitors the motion step (move) of the particle filter.

        The motion step moves particles based on odometric estimates of linear and angular
        velocities.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
        """
        # Move particles based on odometry data
        self._particle_filter.move(z_v, z_w)

        # Visualize move step if enabled
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
