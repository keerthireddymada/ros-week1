import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SubscribeNumber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        self.subscription = self.create_subscription(Int32, 'number', self.receive_message ,10)
    
    def receive_message(self,msg):
        number = msg.data
        square = number * number
        self.get_logger().info(f'The number received = {number} \nThe square of it = {square}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscribeNumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
