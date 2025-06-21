import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')

        self.stop_distance = 0.85  # LIDAR-based distance (chassis end to wall + lidar to chassis end)
        self.lidar_offset = 0.25  # Distance from LIDAR to robot front
        self.latest_cmd = Twist()
        self.obstacle_detected = False

        self.create_subscription(LaserScan, '/scan_laser', self.scan_callback, 10)
        self.create_subscription(Twist, '/input_cmd_vel', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self.publish_safe_cmd)
        self.logged_info = False

    def scan_callback(self, msg):
        if not self.logged_info:
            self.get_logger().info(
                f"LIDAR info: angle_min={msg.angle_min:.2f}, angle_max={msg.angle_max:.2f}, increment={msg.angle_increment:.4f}, total={len(msg.ranges)}"
            )
            self.logged_info = True

        zero_idx = len(msg.ranges) // 2
        angle_window = int((15.0 * 3.1416 / 180.0) / msg.angle_increment)
        start = max(0, zero_idx - angle_window)
        end = min(len(msg.ranges), zero_idx + angle_window)

        front = msg.ranges[start:end]
        valid = [r for r in front if 0.05 < r < float('inf')]

        if valid:
            closest = min(valid)
            distance_from_chassis = max(0.0, closest - self.lidar_offset)

            if closest < self.stop_distance:
                self.obstacle_detected = True
                self.get_logger().info(f"🟥 Obstacle at {distance_from_chassis:.2f} m from chassis front — stopping!")
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def publish_safe_cmd(self):
        output = Twist()
        if self.obstacle_detected and self.latest_cmd.linear.x > 0:
            output.linear.x = 0.0
            output.angular.z = 0.0
        else:
            output = self.latest_cmd
        self.cmd_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
