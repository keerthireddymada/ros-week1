import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

#creating node
class PublishNumber(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher = self.create_publisher(Int32, 'number', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_number)

    def publish_number(self):
        msg = Int32()
        msg.data = 12;
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PublishNumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()  
if __name__ == '__main__':
    main()


