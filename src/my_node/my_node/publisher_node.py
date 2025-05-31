import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#creating node
class PublishNumber(Node):
    def __init__(self):
        super().__init__('number_publisher')
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_number)

        #setting qos
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher = self.create_publisher(Int32, 'number', qos_profile)

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


