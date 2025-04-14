import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from amr_msgs.msg import PoseStamped, WallFollowerActive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import os
import time
import traceback
from transforms3d.euler import euler2quat

from amr_localization.particle_filter import ParticleFilter


class ParticleFilterNode(LifecycleNode):
    def __init__(self):
        """Particle filter node initializer."""
        super().__init__("particle_filter")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("global_localization", True)
        self.declare_parameter("initial_pose", (0.0, 0.0, math.radians(0)))
        self.declare_parameter("initial_pose_sigma", (0.05, 0.05, math.radians(5)))
        self.declare_parameter("particles", 1000)
        self.declare_parameter("sigma_v", 0.1)
        self.declare_parameter("sigma_w", 0.1)
        self.declare_parameter("sigma_z", 0.1)
        self.declare_parameter("steps_btw_sense_updates", 10)
        self.declare_parameter("world", "lab03")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot = self.get_parameter("enable_plot").get_parameter_value().bool_value
            global_localization = (
                self.get_parameter("global_localization").get_parameter_value().bool_value
            )
            initial_pose = tuple(
                self.get_parameter("initial_pose").get_parameter_value().double_array_value.tolist()
            )
            initial_pose_sigma = tuple(
                self.get_parameter("initial_pose_sigma")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            particles = self.get_parameter("particles").get_parameter_value().integer_value
            sigma_v = self.get_parameter("sigma_v").get_parameter_value().double_value
            sigma_w = self.get_parameter("sigma_w").get_parameter_value().double_value
            sigma_z = self.get_parameter("sigma_z").get_parameter_value().double_value
            world = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            self._dt = dt
            self._localized = False
            self._sense_countdown = 2
            self._active = False
            self._movements = []
            self._last_lidar = None
            self._last_pose = None
            self._steps = 0
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._particle_filter = ParticleFilter(
                dt,
                map_path,
                particle_count=particles,
                sigma_v=sigma_v,
                sigma_w=sigma_w,
                sigma_z=sigma_z,
                global_localization=global_localization,
                initial_pose=initial_pose,
                initial_pose_sigma=initial_pose_sigma,
            )

            if self._enable_plot:
                self._particle_filter.show("Initialization", save_figure=True)

            # Publishers
            # TODO: 3.1. Create the /pose publisher (PoseStamped message).
            self._pose_publisher = self.create_publisher(PoseStamped, "/pose", 10)
            self._wall_follower_publisher = self.create_publisher(WallFollowerActive, "/activateFollower", qos_profile=10)
            
            # Subscribers
            scan_qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self.encoder_suscriber = self.create_subscription(Odometry, "/odometry", callback=self._encoder_callback, qos_profile=10)
            self.lidar_subscriber = self.create_subscription(LaserScan, "/scan", callback=self._lidar_callback, qos_profile=scan_qos_profile)

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
        msg = WallFollowerActive()
        msg.active = self._active
        self._wall_follower_publisher.publish(msg)
        self.sense_timer = self.create_timer(self._sense_countdown, self._timer_callback)
        return super().on_activate(state)
    
    def _encoder_callback(self, odom_msg: Odometry):
        z_v: float = odom_msg.twist.twist.linear.x
        z_w: float = odom_msg.twist.twist.angular.z
        if self._last_pose and self._localized:
            x_old = self._last_pose[0]
            y_old = self._last_pose[1]
            theta_old = self._last_pose[2] 

            theta = theta_old + z_w * self._dt
            theta_mean = (theta + theta_old) / 2
            x = x_old + z_v * math.cos(theta_mean) * self._dt
            y = y_old + z_v * math.sin(theta_mean) * self._dt

            self._last_pose = (x, y, theta)
            self._publish_pose_estimate(x, y, theta)
            self.get_logger().info(f"Published pose: {(x,y,theta)}")

            if abs(z_v) > 0.005 or abs(z_w) > 0.005 and self._active:
                self._movements.append((z_v, z_w))
                self.get_logger().info(f"Movements Length: {len(self._movements)}")

    def _lidar_callback(self, msg: LaserScan):
        self._last_lidar = msg
    
    def _timer_callback(self):
        # Cancel and reset are needed because _timer_callback > _sense_countdown
        # Without cancel and reset, the timer callback would be triggered without delay
        self.sense_timer.cancel()

        if self._last_lidar is None:
            self.get_logger().info("No LiDAR measure received.")
            return
        
        self._active = False
        msg = WallFollowerActive()
        msg.active = self._active
        self._wall_follower_publisher.publish(msg)

        for z_v, z_w in self._movements:
            self._execute_motion_step(z_v, z_w)
            self._steps += 1
        self._movements = []
        z_us = self._last_lidar.ranges
        x_h, y_h, theta_h = self._execute_measurement_step(z_us)
 
        self._publish_pose_estimate(x_h, y_h, theta_h)
        self._last_pose = (x_h,y_h,theta_h)

        if not self._localized:
            self._active = True
            msg.active = self._active
            self._wall_follower_publisher.publish(msg)
            self.sense_timer.reset()
        else:
            self.get_logger().info(f"The robot is LOCALIZED")

    def _execute_measurement_step(self, z_us: list[float]) -> tuple[float, float, float]:
        """Executes and monitors the measurement step (sense) of the particle filter.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].

        Returns:
            Pose estimate (x_h, y_h, theta_h) [m, m, rad]; inf if cannot be computed.
        """
        pose = (float("inf"), float("inf"), float("inf"))

        start_time = time.perf_counter()
        self._particle_filter.resample(z_us)
        sense_time = time.perf_counter() - start_time

        self.get_logger().info(f"Sense step time: {sense_time:6.3f} s")

        if self._enable_plot:
            self._particle_filter.show("Sense", save_figure=True)

        start_time = time.perf_counter()
        self._localized, pose = self._particle_filter.compute_pose()
        clustering_time = time.perf_counter() - start_time

        self.get_logger().info(f"Clustering time: {clustering_time:6.3f} s")
 
        return pose

    def _execute_motion_step(self, z_v: float, z_w: float):
        """Executes and monitors the motion step (move) of the particle filter.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
        """
        start_time = time.perf_counter()
        self._particle_filter.move(z_v, z_w)
        move_time = time.perf_counter() - start_time

        self.get_logger().info(f"Move step time: {move_time:7.3f} s")

        if self._enable_plot:
            self._particle_filter.show("Move", save_figure=True)

    def _publish_pose_estimate(self, x_h: float, y_h: float, theta_h: float) -> None:
        """Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].

        """
        # TODO: 3.2. Complete the function body with your code (i.e., replace the pass statement).
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        if self._localized:
            pose_msg.pose.position.x = x_h
            pose_msg.pose.position.y = y_h
            
            q = euler2quat(0.0, 0.0, theta_h)
            pose_msg.pose.orientation.x = q[1]
            pose_msg.pose.orientation.y = q[2]
            pose_msg.pose.orientation.z = q[3]
            pose_msg.pose.orientation.w = q[0]
        pose_msg.localized = self._localized

        self._pose_publisher.publish(pose_msg)
        
def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = ParticleFilterNode()

    try:
        rclpy.spin(particle_filter_node)
    except KeyboardInterrupt:
        pass

    particle_filter_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
