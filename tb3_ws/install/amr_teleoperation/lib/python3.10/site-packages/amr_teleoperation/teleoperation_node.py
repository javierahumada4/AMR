import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from amr_msgs.msg import Key
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class KeySubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Key,
            'key',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()

    def listener_callback(self, msg):
        # Log the received key
        self.get_logger().info(f'I heard: "{msg.key}"')
        
        # Update velocities based on key input
        if msg.key == 'w':  # Move forward
            self.twist.linear.x += 0.1
        elif msg.key == 's':  # Move backward
            self.twist.linear.x -= 0.1
        elif msg.key == 'a':  # Turn left
            self.twist.angular.z += 0.1
        elif msg.key == 'd':  # Turn right
            self.twist.angular.z -= 0.1
        elif msg.key == 'space':  # Stop (spacebar)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            return

        # Publish the updated Twist message to the /cmd_vel topic
        self.publisher.publish(self.twist)
        
        # Log the published velocities for debugging purposes
        self.get_logger().info(f"Published velocities: linear={self.twist.linear.x}, angular={self.twist.angular.z}")    
        
class LiDARSubscriber(Node):
    
    def __init__(self):
        super().__init__('lidar_subscriber')

        # Publicador para enviar comandos de velocidad al TurtleBot3
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptor al LiDAR con un perfil de QoS adecuado para datos de sensores
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, 
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.lidar_callback,
            qos_profile
        )
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Inicialización de variables
        self.stop_distance = 0.2  # Minimum safe distance
        self.current_twist = Twist()
        self.logger_timer = 0

        self.get_logger().info("Teleoperation node initialized.")

    def cmd_vel_callback(self, msg):
        self.current_twist = msg

    def lidar_callback(self, msg):
        front = msg.ranges[0:20] + msg.ranges[-20:]
        back = msg.ranges[100:140] 

        safe_to_move_forward = all(d > self.stop_distance for d in front if d > 0)
        safe_to_move_backward = all(d > self.stop_distance for d in back if d > 0)    

        stop_twist = Twist()
        stop_twist.linear = self.current_twist.linear
        stop_twist.angular = self.current_twist.angular

        if not safe_to_move_forward and self.current_twist.linear.x > 0:
            self.current_twist.linear.x = 0.0
        if not safe_to_move_backward and self.current_twist.linear.x < 0:
            self.current_twist.linear.x = 0.0
        
        self.cmd_vel_publisher.publish(self.current_twist)
        
        self.logger_timer += 1
        if self.logger_timer % 10 == 0:
            self.get_logger().info(f"Safe forward: {safe_to_move_forward}, Safe backward: {safe_to_move_backward}")
            self.get_logger().info(f"Published velocities: linear={self.current_twist.linear.x}, angular={self.current_twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)

    key_subscriber = KeySubscriber()
    lidar_subscriber = LiDARSubscriber()

    # Use a MultiThreadedExecutor to spin both nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(key_subscriber)
    executor.add_node(lidar_subscriber)

    try:
        executor.spin()
    finally:
        key_subscriber.destroy_node()
        lidar_subscriber.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()