import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublishHello(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher = self.create_publisher(String , 'greetings', 10)
        self.timer = self.create_timer(1.0 ,self.publish_hello)

    def publish_hello(self):
        msg = String()
        msg.data = 'Hello'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = PublishHello()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


