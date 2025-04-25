# Importing necessary ROS 2 libraries
import rclpy
from rclpy.lifecycle import (
    LifecycleNode,
    LifecycleState,
    TransitionCallbackReturn,
)

# Importing message types
from amr_msgs.msg import PoseStamped  # Custom message type for pose
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path

# Importing additional libraries
import math  # For mathematical operations
import traceback  # For error handling and debugging
from transforms3d.euler import quat2euler  # For quaternion to Euler angle conversion
from typing import List, Tuple

# Importing custom pure pursuit implementation
from amr_control.pure_pursuit import PurePursuit


class PurePursuitNode(LifecycleNode):
    """
    A ROS 2 Lifecycle Node implementing a pure pursuit controller.

    This node subscribes to the robot's pose and a path, computes velocity commands (linear and angular),
    and publishes them to control the robot's motion.
    """

    def __init__(self) -> None:
        """Pure pursuit node initializer."""
        super().__init__("pure_pursuit")

        # Declare parameters with default values
        self.declare_parameter("dt", 0.05)  # Time step for the controller
        self.declare_parameter(
            "lookahead_distance", 0.3
        )  # Lookahead distance for pure pursuit

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handles the 'configuring' transition in the node's lifecycle.

        Initializes parameters, sets up subscriptions and publishers, and prepares the pure pursuit controller.

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
            lookahead_distance: float = (
                self.get_parameter("lookahead_distance")
                .get_parameter_value()
                .double_value
            )

            # Create subscriptions for pose and path topics
            self._subscriber_pose = self.create_subscription(
                PoseStamped, "/pose", self._compute_commands_callback, 10
            )
            self._subscriber_path = self.create_subscription(
                Path, "/path", self._path_callback, 10
            )

            # Create a publisher for velocity commands
            self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)

            # Initialize the pure pursuit controller
            self._pure_pursuit: PurePursuit = PurePursuit(dt, lookahead_distance)

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

    def _compute_commands_callback(self, pose_msg: PoseStamped) -> None:
        """
        Callback function for the pose subscriber.

        Computes velocity commands using the pure pursuit algorithm based on the robot's current pose,
        and publishes them as TwistStamped messages.

        Args:
            pose_msg: Message containing the estimated robot pose.
        """
        if pose_msg.localized:  # Ensure the robot is localized before proceeding
            # Extract position (x, y) and orientation (quaternion) from the pose message
            x: float = pose_msg.pose.position.x
            y: float = pose_msg.pose.position.y
            quat_w: float = pose_msg.pose.orientation.w
            quat_x: float = pose_msg.pose.orientation.x
            quat_y: float = pose_msg.pose.orientation.y
            quat_z: float = pose_msg.pose.orientation.z

            # Convert quaternion to Euler angles
            _, _, theta = quat2euler((quat_w, quat_x, quat_y, quat_z))
            theta %= 2 * math.pi  # Normalize theta to [0, 2pi]

            # Compute linear (v) and angular (w) velocity commands using pure pursuit
            v, w = self._pure_pursuit.compute_commands(x, y, theta)

            # Publish velocity commands to the /cmd_vel topic
            self._publish_velocity_commands(v, w)

    def _path_callback(self, path_msg: Path) -> None:
        """
        Callback function for the path subscriber.

        Saves the path that the pure pursuit controller will follow.

        Args:
            path_msg: Message containing the (smoothed) path as a sequence of poses.
        """
        path: List[Tuple[float, float]] = (
            []
        )  # Initialize an empty list to store path points

        for pose in path_msg.poses:
            x: float = pose.pose.position.x
            y: float = pose.pose.position.y
            path.append((x, y))  # Append each (x, y) point to the path list

        # Update the path in the pure pursuit controller object
        self._pure_pursuit.path = path

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """
        Publishes velocity commands as a TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].
        """
        msg = TwistStamped() # Create a new TwistStamped message instance

        # Set timestamp in header to current time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set linear and angular velocities
        msg.twist.linear.x = v
        msg.twist.angular.z = w

        # Publish the message to the /cmd_vel topic
        self._publisher.publish(msg)


def main(args=None) -> None:
    """
    Main entry point for running this node.

    Initializes ROS 2 communication and spins until interrupted.
    """
    rclpy.init(args=args)  # Initialize ROS communication

    pure_pursuit_node = PurePursuitNode()  # Create an instance of PurePursuitNode

    try:
        rclpy.spin(
            pure_pursuit_node
        )  # Keep spinning until shutdown or interruption occurs
    except KeyboardInterrupt:
        pass

    pure_pursuit_node.destroy_node()  # Clean up resources by destroying the node instance
    rclpy.try_shutdown()  # Shut down ROS communication


if __name__ == "__main__":
    main()
