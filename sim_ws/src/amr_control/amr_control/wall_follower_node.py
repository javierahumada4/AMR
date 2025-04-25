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
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import traceback
from typing import List, Optional

from amr_control.wall_follower import WallFollower  # Custom wall-following controller


class WallFollowerNode(LifecycleNode):
    """
    A ROS 2 Lifecycle Node implementing a wall-following controller.

    This node subscribes to odometry, LiDAR scan data, and optionally pose data
    (if localization is enabled), computes velocity commands using a wall-following algorithm,
    and publishes these commands.
    """

    def __init__(self) -> None:
        """Wall follower node initializer."""
        super().__init__("wall_follower")

        # Declare parameters with default values
        self.declare_parameter("dt", 0.05)  # Sampling period for the controller
        self.declare_parameter(
            "enable_localization", False
        )  # Whether to use localization data

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handles the 'configuring' transition in the node's lifecycle.

        Initializes parameters, sets up subscriptions and publishers, and prepares
        the wall-following controller.

        Args:
            state: Current lifecycle state.

        Returns:
            TransitionCallbackReturn: Result of the transition.
        """
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )

        try:
            dt: float = self.get_parameter("dt").get_parameter_value().double_value
            enable_localization: bool = (
                self.get_parameter("enable_localization")
                .get_parameter_value()
                .bool_value
            )

            # Set up QoS profile for subscriptions from the LiDAR
            qos_profile: QoSProfile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
            )

            # Create synchronized subscribers for odometry and LiDAR scan data
            self._subscribers: List[message_filters.Subscriber] = []
            self._subscribers.append(
                message_filters.Subscriber(self, Odometry, "/odometry", qos_profile=10)
            )
            self._subscribers.append(
                message_filters.Subscriber(
                    self, LaserScan, "/scan", qos_profile=qos_profile
                )
            )

            # Add pose subscription if localization is enabled
            if enable_localization:
                self._subscribers.append(
                    message_filters.Subscriber(
                        self, PoseStamped, "/pose", qos_profile=10
                    )
                )

            # Synchronize messages from subscribed topics using an approximate time synchronizer
            ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers, queue_size=10, slop=9
            )
            ts.registerCallback(self._compute_commands_callback)

            # Create a publisher for velocity commands (TwistStamped messages)
            self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)

            # Initialize the wall-following controller object with the sampling period
            self._wall_follower: WallFollower = WallFollower(dt)

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

    def _compute_commands_callback(
        self,
        odom_msg: Odometry,
        scan_msg: LaserScan,
        pose_msg: Optional[PoseStamped] = None,
    ) -> None:
        """
        Callback function for synchronized subscribers.

        Executes the wall-following controller using odometry and LiDAR data,
        and optionally pose data (if localization is enabled). Publishes computed velocity commands.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR readings.
            pose_msg: Message containing the estimated robot pose (optional).
        """
        if (
            pose_msg is None or not pose_msg.localized
        ):  # Only operate if localization is not available or not required
            # Parse linear and angular velocities from odometry message
            z_v: float = odom_msg.twist.twist.linear.x  # Linear velocity [m/s]
            z_w: float = odom_msg.twist.twist.angular.z  # Angular velocity [rad/s]

            # Parse LiDAR measurements from LaserScan message (ranges array)
            z_scan: List[float] = scan_msg.ranges

            # Compute linear (v) and angular (w) velocity commands using wall follower logic
            v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)

            # Publish computed velocity commands to the /cmd_vel topic
            self._publish_velocity_commands(v, w)

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """
        Publishes velocity commands as a TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].
        """
        msg: TwistStamped = TwistStamped()  # Create a new TwistStamped message instance

        # Set the linear and angular velocities in the message
        msg.twist.linear.x = v
        msg.twist.angular.z = w

        # Set timestamp in header to current time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message to the /cmd_vel topic
        self.publisher.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for running this node.

    Initializes ROS 2 communication and spins until interrupted.
    """
    rclpy.init(args=args)  # Initialize ROS communication

    wall_follower_node: WallFollowerNode = (
        WallFollowerNode()
    )  # Create an instance of WallFollowerNode

    try:
        rclpy.spin(
            wall_follower_node
        )  # Keep spinning until shutdown or interruption occurs
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()  # Clean up resources by destroying the node instance
    rclpy.try_shutdown()  # Shut down ROS communication


if __name__ == "__main__":
    main()
