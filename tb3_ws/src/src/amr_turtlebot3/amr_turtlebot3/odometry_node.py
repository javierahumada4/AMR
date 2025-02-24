#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovariance
from transforms3d.euler import quat2euler
import math
import traceback


class OdometryNode(LifecycleNode):
    def __init__(self):
        super().__init__('odometry_node')
        # self.subscription = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback, 
        #     10
        # )
        # self.publisher = self.create_publisher(Odometry, '/odometry', 10)

        # Variables para almacenar la última posición y tiempo
        self.last_x = None
        self.last_y = None
        self.last_theta = None
        self.last_time = None
        
        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_localization", False)

    def odom_callback(self, msg):
        # Extraer posición actual del mensaje de odometría
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Convertir cuaternión a ángulo de Euler (yaw)
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        _, _, current_theta = quat2euler(quaternion, axes='sxyz')  # Convención estándar ZYX

        # Obtener el tiempo actual del mensaje
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Si no hay datos previos, inicializar y salir
        if self.last_time is None:
            self.last_x = current_x
            self.last_y = current_y
            self.last_theta = current_theta
            self.last_time = current_time
            return

        # Calcular diferencias en posición, orientación y tiempo
        delta_x = current_x - self.last_x
        delta_y = current_y - self.last_y
        delta_theta = current_theta - self.last_theta

        # Normalizar delta_theta para que esté entre -pi y pi
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

        delta_time = current_time - self.last_time

        if delta_time > 0:
            # Calcular velocidades lineales y angulares
            linear_velocity = math.sqrt(delta_x**2 + delta_y**2) / delta_time
            angular_velocity = delta_theta / delta_time

            # Publicar las velocidades calculadas en el tópico /odometry
            msg.twist.twist.linear.x = linear_velocity
            msg.twist.twist.angular.z = angular_velocity
            self.publisher.publish(msg)

        # Actualizar los valores previos para la próxima iteración
        self.last_x = current_x
        self.last_y = current_y
        self.last_theta = current_theta
        self.last_time = current_time
        
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
            
            self.subscription = self.create_subscription(
                msg_type=Odometry,
                topic = "odom",
                callback = self.odom_callback,
                qos_profile = 10,    
            )
            
            self.publisher = self.create_publisher(
                msg_type = Odometry,
                topic = "/odometry",
                qos_profile = 10,   
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

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()

    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass

    odometry_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()