import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import message_filters
from amr_msgs.msg import PoseStamped, WallFollowerActive 
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import traceback

from amr_control.wall_follower import WallFollower


class WallFollowerNode(LifecycleNode):
    def __init__(self):
        """Wall follower node initializer."""
        super().__init__("wall_follower")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_localization", False)
        self.active = True

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            enable_localization = (
                self.get_parameter("enable_localization").get_parameter_value().bool_value
            )

            # Subscribers
            # TODO: 2.7. Synchronize _compute_commands_callback with /odometry and /scan.
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
            )
            self._subscribers: list[message_filters.Subscriber] = []
            # Append as many topics as needed
            self._subscribers.append(
                message_filters.Subscriber(self, Odometry, "/odometry", qos_profile=10)
            )
            self._subscribers.append(
                message_filters.Subscriber(self, LaserScan, "/scan", qos_profile=qos_profile)
            )
            ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers, queue_size=10, slop=0.25, allow_headerless=True #0.1 o 0.2 max 0.5
            )
            ts.registerCallback(self._compute_commands_callback)
            
            self.wall_follower_active_suscriber = self.create_subscription(WallFollowerActive, "/activateFollower", callback=self._wall_follower_active_callback, qos_profile=10)
                        
            # TODO: 4.12. Add /pose to the synced subscriptions only if localization is enabled.
            if enable_localization:
                self.pose_subscriber = self.create_subscription(PoseStamped, "/pose", qos_profile=10)
                
            # Publishers
            # TODO: 2.10. Create the /cmd_vel velocity commands publisher (Twist message).
            self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

            # Attribute and object initializations
            self._wall_follower = WallFollower(dt)

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)

    def _wall_follower_active_callback(
        self, active_msg: WallFollowerActive = WallFollowerActive()
    ):
        self.active = active_msg.active
    
    def _compute_commands_callback(
        self, odom_msg: Odometry, scan_msg: LaserScan, pose_msg: PoseStamped = PoseStamped()
    ):
        """Subscriber callback. Executes a wall-following controller and publishes v and w commands.

        Ceases to operate once the robot is localized.n

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR readings.
            pose_msg: Message containing the estimated robot pose.

        """
        
        if self.active:
            if not pose_msg.localized:
                # TODO: 2.8. Parse the odometry from the Odometry message (i.e., read z_v and z_w).
                z_v: float = odom_msg.twist.twist.linear.x
                z_w: float = odom_msg.twist.twist.angular.z

                # TODO: 2.9. Parse LiDAR measurements from the LaserScan message (i.e., read z_scan).
                z_scan: list[float] = scan_msg.ranges    
                v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
                
                self.get_logger().info(f"Commands: v = {v:.3f} m/s, w = {w:+.3f} rad/s")
                # Publish
            else:
                v, w = 0.0, 0.0      
        else:
            v, w = 0.0, 0.0
        self._publish_velocity_commands(v, w)   

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.Twist message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """
        # TODO: 2.11. Complete the function body with your code (i.e., replace the pass statement).
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
