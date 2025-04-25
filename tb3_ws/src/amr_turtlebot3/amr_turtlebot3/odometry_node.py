#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math
import traceback


class OdometryNode(LifecycleNode):
    """Node to calculate and publish odometry information.

    It receives odometry messages from the robot, calculates the linear 
    and angular velocities based on the current and previous positions and orientations, 
    and publishes the calculated velocities in the /odometry topic.
    """

    def __init__(self):
        super().__init__("odometry_node")

        # Storage for previous values of position, orientation and time
        self.last_x = None
        self.last_y = None
        self.last_theta = None
        self.last_time = None

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_localization", False)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )

        try:
            # Create a subscription to the odometry topic from the robot
            self.subscription = self.create_subscription(
                msg_type=Odometry,
                topic="odom",
                callback=self.odom_callback,
                qos_profile=10,
            )

            # Create a publisher for the calculated odometry for other nodes
            self.publisher = self.create_publisher(
                msg_type=Odometry,
                topic="/odometry",
                qos_profile=10,
            )

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_activate(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)

    def odom_callback(self, msg):
        """
        Callback function to process odometry messages.
        It calculates the linear and angular velocities based on the current and previous positions and orientations.
        It publishes the calculated velocities in the /odometry topic.

        Args:
            msg: Odometry message containing the current position and orientation of the robot.
        """
        # Parse the current position from the message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Convert quaternion to Euler angle (yaw)
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.w,
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
        ]
        _, _, current_theta = quat2euler(
            quaternion, axes="sxyz"
        )  # Standard ZYX convention

        # Get the current time from the message
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # If there are no previous data, initialize and exit
        if self.last_time is None:
            self.last_x = current_x
            self.last_y = current_y
            self.last_theta = current_theta
            self.last_time = current_time
            return

        # Calculate differences in position, orientation, and time
        delta_x = current_x - self.last_x
        delta_y = current_y - self.last_y
        delta_theta = current_theta - self.last_theta

        # Normalize delta_theta to be between -pi and pi
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

        # Calculate the time difference
        delta_time = current_time - self.last_time

        if delta_time > 0:
            # Calculate linear and angular velocities
            linear_velocity = math.sqrt(delta_x**2 + delta_y**2) / delta_time
            angular_velocity = delta_theta / delta_time

            # Publish the calculated velocities to the /odometry topic
            msg.twist.twist.linear.x = linear_velocity
            msg.twist.twist.angular.z = angular_velocity
            self.publisher.publish(msg)

        # Update the previous values for the next iteration
        self.last_x = current_x
        self.last_y = current_y
        self.last_theta = current_theta
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()

    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass

    odometry_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
