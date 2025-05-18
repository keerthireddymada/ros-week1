import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class SquareMovement(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_square)

        self.side_length = 2.0
        self.linear_speed = 0.5
        self.angular_speed = pi/4

        self.state = 'FORWARD'
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        self.sides_completed = 0

    def move_square(self):
        msg = Twist()

        if self.state == 'FORWARD':
            msg.linear.x = self.linear_speed
            self.distance_traveled += self.linear_speed * 0.1
        
            if self.distance_traveled >= self.side_length:
                self.state = 'TURN'
                self.distance_traveled = 0
            
        elif self.state == 'TURN':
            msg.angular.z = self.angular_speed
            self.angle_turned += abs(self.angular_speed) * 0.1

            if self.angle_turned >= pi/2:  
                self.state = 'FORWARD'
                self.angle_turned = 0.0
                self.sides_completed += 1
                
                if self.sides_completed >= 4:
                    self.sides_completed = 0
                    self.get_logger().info("Completed one square!")    
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = SquareMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()