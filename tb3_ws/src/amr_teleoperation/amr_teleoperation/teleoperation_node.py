import rclpy
from rclpy.node import Node

from amr_msgs.msg import Key
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class KeySubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Key,
            'key',
            self.on_key_press,
            10)
        self.subscription  # prevent unused variable warning
        self.twist = Twist()

    def on_key_press(self, key):
        if key.char == 'w':  # Adelante
            self.twist.linear.x += 0.1
        elif key.char == 's':  # Atrás
            self.twist.linear.x -= 0.1
        elif key.char == 'a':  # Girar a la derecha
            self.twist.angular.z += 0.1
        elif key.char == 'd':  # Girar a la izquierda
            self.twist.angular.z -= 0.1
        
        # Detener el robot con la barra espaciadora
        elif key == ' ':
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            return

        # Publicar las velocidades actualizadas
        self.publisher.publish(self.twist)
        self.get_logger().info(f"Published velocities: linear={self.twist.linear.x}, angular={self.twist.angular.z}")
          

class LiDARSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')

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
            '/scan',  # Cambiar al nombre del tópico LiDAR si es diferente
            self.lidar_callback,
            qos_profile
        )

        # Inicialización de variables
        self.safe_to_move = True  # Bandera para indicar si es seguro moverse
        self.stop_distance = 0.5  # Distancia mínima para detenerse (en metros)
        self.current_twist = Twist()  # Último comando de velocidad

        self.get_logger().info("Nodo de teleoperación segura iniciado.")

    def lidar_callback(self, msg):
        for distance in msg.ranges:
            if distance < self.stop_distance and distance > 0.0:
                self.safe_to_move = False
                self.get_logger().warn("¡Obstáculo detectado! Deteniendo el robot.")
                stop_twist = Twist()
                self.cmd_vel_publisher.publish(stop_twist)
                return
        
        self.safe_to_move = True

def main(args=None):
    rclpy.init(args=args)

    key_subscriber = KeySubscriber()
    lidar_subscriber = LiDARSubscriber()


    rclpy.spin(key_subscriber)
    rclpy.spin(lidar_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_subscriber.destroy_node()
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()