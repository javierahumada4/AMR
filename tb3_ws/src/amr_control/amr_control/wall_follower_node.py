import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

import message_filters
from amr_msgs.msg import (
    PoseStamped,
    WallFollowerActive,
)  # Custom messages type for pose and wall follower activation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import traceback
from typing import List

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
        self.active: bool = True  # Flag to indicate if the wall follower is active

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
            # Parameters
            dt: float = self.get_parameter("dt").get_parameter_value().double_value

            # Attribute and object initializations
            self._wall_follower: WallFollower = WallFollower(dt)

            # Create a publisher for velocity commands (Twist messages)
            self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

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

            # Store the last velocity commands
            self._last_v: float = 50.0
            self._last_w: float = 50.0

            # Synchronize messages from subscribed topics using an approximate time synchronizer
            ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers,
                queue_size=10,
                slop=0.25,
                allow_headerless=True,
            )
            ts.registerCallback(self._compute_commands_callback)

            # Create a subscription for checking wall follower activation
            self.wall_follower_active_suscriber = self.create_subscription(
                WallFollowerActive,
                "/activateFollower",
                callback=self._wall_follower_active_callback,
                qos_profile=10,
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
        return super().on_activate(state)

    def _wall_follower_active_callback(
        self, active_msg: WallFollowerActive = WallFollowerActive()
    ) -> None:
        """
        Callback for the wall follower activation message.
        """
        self.active = active_msg.active  # Update the active state based on the message

    def _compute_commands_callback(
        self,
        odom_msg: Odometry,
        scan_msg: LaserScan,
        pose_msg: PoseStamped = PoseStamped(),
    ) -> None:
        """Subscriber callback. Executes a wall-following controller and publishes v and w commands.

        Ceases to operate once the robot is localized.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR readings.
            pose_msg: Message containing the estimated robot pose.

        """
        if self.active:  # Check if the wall follower is active
            if not pose_msg.localized:  # Check if the robot is localized
                # Parse linear and angular velocities from odometry message
                z_v: float = odom_msg.twist.twist.linear.x  # Linear velocity [m/s]
                z_w: float = odom_msg.twist.twist.angular.z  # Angular velocity [rad/s]

                # Parse LiDAR measurements from LaserScan message (ranges array)
                z_scan: List[float] = scan_msg.ranges

                # Compute linear (v) and angular (w) velocity commands using wall follower logic
                v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
            else:
                v, w = 0.0, 0.0
        else:
            # If the wall follower is not active, set v and w to 0.0
            # But only if the last commands were not 0.0
            # This prevents the robot from receiving 0.0 commands if it was already stopped
            if self._last_v != 0.0 or self._last_w != 0.0:
                v, w = 0.0, 0.0
            else:
                return  # No need to publish if the last commands were already 0.0

        # Update the last velocity commands
        self._last_v = v
        self._last_w = w

        # Publish computed velocity commands to the /cmd_vel topic
        self._publish_velocity_commands(v, w)

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.Twist message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """
        msg: Twist = Twist()  # Create a new Twist message instance

        # Set the linear and angular velocities in the message
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        # Publish the message to the /cmd_vel topic
        self.publisher.publish(msg)


def main(args=None) -> None:
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
