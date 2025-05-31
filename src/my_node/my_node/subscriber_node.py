import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class SubscribeNumber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(Int32, 'number', self.receive_message, qos_profile)
    
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
