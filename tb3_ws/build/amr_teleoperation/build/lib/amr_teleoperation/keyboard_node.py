import rclpy
from rclpy.node import Node

from amr_msgs.msg import Key

from sshkeyboard import listen_keyboard

class KeyPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Key, 'key', 10)
        listen_keyboard(
            on_press=self.press,
            delay_second_char=0.75,
            delay_other_chars=0.05,
        )
        
    def press(self, key):
        msg = Key()
        msg.key = f"{key}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = KeyPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
