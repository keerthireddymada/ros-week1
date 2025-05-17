# ros-week1
I’ll provide the complete raw Markdown content for the GitHub README as a single block, including all sections: ROS 2 basics (theory), turtlesim, and Python nodes. As per your previous request, commands are grouped into separate "Commands" blocks, the "Moving the Turtle in a Square (Using an Action)" section is removed, and the content remains beginner-friendly with a focus on theory. This is the exact text you can copy and paste into your `README.md` file on GitHub.

---

### Raw Markdown Content

```
# ROS 2 Basics for Beginners

This README provides a beginner-friendly introduction to ROS 2 (Robot Operating System 2), a framework for building robot applications. We'll cover the core concepts, set up a simple workspace, explore the turtlesim simulator, and create basic Python nodes to understand how ROS 2 works. Everything is explained at a noob level for crystal-clear understanding.

## 1. ROS 2 Basics (Theory)

### What is ROS 2?
ROS 2 is a system that helps different parts of a robot (or robot software) talk to each other. Imagine a robot as a team of workers: each worker has a specific job, and they need to share information to get the task done. ROS 2 makes this communication easy by providing tools and rules for these "workers" to collaborate.

### Key Concepts in ROS 2

#### Nodes
Nodes are like small programs in ROS 2. Each node has a specific job, like controlling a motor, reading a sensor, or drawing a map. These nodes run at the same time and talk to each other. For example, one node might move a robot forward, while another node uses a camera to detect obstacles.

#### Topics
Topics are like chat rooms where nodes share messages. A node can "publish" (send) messages to a topic, and other nodes can "subscribe" (listen) to that topic to receive those messages. Publisher nodes send messages (like a sensor sending the robot's speed), and subscriber nodes listen to those messages (like a navigation system using the speed to decide where to go). When creating a topic, you specify its name and the type of message it will carry, like a number or text.

#### Services
Services are like a direct conversation between two nodes. One node asks for something (like "move to this position"), and the other node replies (like "done!"). Unlike topics, where publishers don’t wait for a reply, services involve waiting for a response.

#### Parameters
Parameters are like settings for a node that you can change while the node is running. For example, you might adjust a robot’s speed without stopping it. You can modify these settings using a special flag when running a node.

#### Remapping
Remapping lets you change the name of a topic. This is helpful when two nodes need to communicate but their topic names don’t match. It’s a way to avoid conflicts or connect nodes that use different topic names by renaming topics when you run the node.

#### Commands for ROS 2 Concepts
Here are the key commands to work with nodes, topics, services, parameters, and remapping:
```
# Run a node
ros2 run <package_name> <node_name>

# See all running nodes
ros2 node list

# See all available topics
ros2 topic list

# View messages in a topic (like a subscriber)
ros2 topic echo <topic_name>

# See all available services
ros2 service list

# Call a service manually
ros2 service call <service_name> <service_message>

# Change parameters when running a node
--ros-args -p

# Remap a topic when running a node
--ros-args -r
```

### Setting Up a Workspace
Before we can create nodes, we need a workspace—a folder where our ROS 2 projects live. This is like setting up a desk where you’ll do all your robot programming work.

#### Commands for Workspace Setup
```
# Create a workspace and a src folder
mkdir -p ~/my_ws/src
cd ~/my_ws

# Build the workspace
colcon build

# Add the source command to .bashrc for auto-sourcing
echo "source ~/my_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Creating a Package
A package is like a container for your nodes and other ROS 2 files. It helps keep your project organized.

#### Commands for Creating a Package
```
# Go to the src folder
cd ~/my_ws/src

# Create a package called my_node
ros2 pkg create --build-type ament_python my_node

# Go to the my_node package and create an __init__.py file
cd ~/my_ws/src/my_node/my_node
touch __init__.py
```

## 2. Exploring Turtlesim

### What is Turtlesim?
Turtlesim is a simple simulator in ROS 2 that shows a turtle on a blue screen. You can move the turtle around, making it a great way to learn ROS 2 concepts like topics and services. It’s like a sandbox where you can practice controlling a robot.

### Installing and Running Turtlesim
First, you need to install turtlesim and run it to see the turtle on the screen. You can then use keyboard controls to move the turtle around.

### Using Services to Teleport the Turtle
You can instantly move the turtle to a new position using a service. This is like telling the turtle to "jump" to a specific spot on the screen with a specific angle.

#### Commands for Turtlesim
```
# Install turtlesim (for ROS Humble)
sudo apt install ros-humble-turtlesim

# Check if turtlesim is installed
ros2 pkg executables turtlesim

# Run the turtlesim node (shows the turtle)
ros2 run turtlesim turtlesim_node

# Move the turtle using keyboard controls (in a new terminal)
ros2 run turtlesim turtle_teleop_key

# Teleport the turtle to position (3, 3) with a 90-degree angle
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 3.0, y: 3.0, theta: 1.57}"
```

### Visualizing the Turtle’s Movement
When you run the turtle controller node (shown later), the turtle will move in a square, leaving a trail behind it. If you have a screenshot of this (e.g., `turtlesim_movement.png`), you can add it to your repository and link it here:
```
![Turtle moving in a square](turtlesim_movement.png)
```

## 3. Python Nodes

### Creating Publisher and Subscriber Nodes
Let’s create two nodes: a publisher that sends a number and a subscriber that receives it and calculates its square.

#### Publisher Node (`publisher_node.py`)
This node sends a number (12) to a topic called `number` every second. It’s like a worker who keeps shouting a number to anyone listening.

1. Create the file:
   ```bash
   cd ~/my_ws/src/my_node/my_node
   touch publisher_node.py
   ```
2. Add this code to `publisher_node.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Int32

   class PublishNumber(Node):
       def __init__(self):
           super().__init__('number_publisher')
           self.publisher = self.create_publisher(Int32, 'number', 10)
           timer_period = 1.0
           self.timer = self.create_timer(timer_period, self.publish_number)

       def publish_number(self):
           msg = Int32()
           msg.data = 12
           self.publisher.publish(msg)

   def main(args=None):
       rclpy.init(args=args)
       node = PublishNumber()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
3. **What does this code do?**
   - `rclpy.init(args=args)`: Starts the ROS 2 system.
   - `self.create_publisher(Int32, 'number', 10)`: Creates a publisher that sends `Int32` messages to the `number` topic.
   - `self.create_timer(timer_period, self.publish_number)`: Calls the `publish_number` function every second.
   - `rclpy.spin(node)`: Keeps the node running so the timer works.
   - `node.destroy_node()` and `rclpy.shutdown()`: Clean up when the node stops.

#### Subscriber Node (`subscriber_node.py`)
This node listens to the `number` topic, calculates the square of the number, and prints it. It’s like a worker who listens to the shouted number and does some math with it.

1. Create the file:
   ```bash
   touch subscriber_node.py
   ```
2. Add this code to `subscriber_node.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Int32

   class SubscribeNumber(Node):
       def __init__(self):
           super().__init__('number_subscriber')
           self.subscription = self.create_subscription(Int32, 'number', self.receive_message, 10)

       def receive_message(self, msg):
           number = msg.data
           square = number * number
           self.get_logger().info(f'The number received = {number}, the square of it = {square}')

   def main(args=None):
       rclpy.init(args=args)
       node = SubscribeNumber()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
3. **What does this code do?**
   - `self.create_subscription(Int32, 'number', self.receive_message, 10)`: Subscribes to the `number` topic and calls `receive_message` when a message arrives.
   - `self.get_logger().info(...)`: Prints the received number and its square.

#### Build and Run the Nodes
Here are the commands to build and run your publisher and subscriber nodes:

#### Commands for Publisher and Subscriber Nodes
```
# Go to the workspace and build
cd ~/my_ws
colcon build

# Source the workspace
source install/setup.bash

# Run the publisher node
ros2 run my_node publisher

# Run the subscriber node (in a new terminal)
ros2 run my_node subscriber
```

#### Debugging Tip
To see what the publisher is sending, add this line in the `publish_number` function:
```python
print(f"Publishing: {msg.data}")
```
Then rebuild and run. You can also check the topic to see what’s happening.

#### Commands for Debugging
```
# Rebuild after adding the debug print
colcon build --packages-select my_node
source install/setup.bash
ros2 run my_node publisher

# List all topics
ros2 topic list

# Check the type of the number topic
ros2 topic info /number
```

### Turtle Controller Node (`square_mover.py`)
This node makes the turtle in turtlesim move in a square. It’s like giving the turtle a set of instructions to draw a square shape.

1. Create the file in the `turtle_controller` package:
   ```bash
   cd ~/my_ws/src/turtle_controller/turtle_controller
   touch square_mover.py
   ```
2. Add this code to `square_mover.py`:
   ```python
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
   ```
3. **What does this code do?**
   - `self.create_publisher(Twist, '/turtle1/cmd_vel', 10)`: Publishes `Twist` messages to control the turtle’s movement.
   - `self.create_timer(0.1, self.move_square)`: Calls `move_square` every 0.1 seconds.
   - The node alternates between moving forward (`FORWARD` state) and turning (`TURN` state) to draw a square with a side length of 2.0 units.
   - After completing four sides, it logs “Completed one square!” and starts over.

#### Commands to Run the Turtle Controller
```
# Build and source the workspace (as shown above)
# Run turtlesim
ros2 run turtlesim turtlesim_node

# Run the turtle controller (in a new terminal)
ros2 run turtle_controller square_mover
```

## Additional Resources
For more ROS 2 basics, check out this tutorial: [Articulated Robotics ROS Overview](https://articulatedrobotics.xyz/tutorials/ready-for-ros/ros-overview).

---

This README covers the essentials of ROS 2 at a beginner level. Start by understanding the theory, play with turtlesim, and then try the Python nodes to see ROS 2 in action!
```

---

### Outside the Raw File (Additional Notes)
The raw Markdown content above is everything you need for your `README.md` file. However, here are a few additional notes that aren’t part of the README but might help you:

- **Image File**: The README references a `turtlesim_movement.png` image for the turtlesim section. You’ll need to add this image to your GitHub repository in the same directory as the `README.md` file for the link (`![Turtle moving in a square](turtlesim_movement.png)`) to work. If you don’t have the image or prefer not to include it, you can remove that line from the README.
- **File Structure**: The README assumes you’re creating the nodes (`publisher_node.py`, `subscriber_node.py`, `square_mover.py`) in specific packages (`my_node` and `turtle_controller`). Make sure your repository matches this structure, or adjust the paths in the README accordingly.
- **ROS Version**: The commands (e.g., `ros-humble-turtlesim`) are based on ROS Humble, as mentioned in your inputs. If you’re using a different ROS 2 distribution (e.g., Foxy, Iron), you’ll need to adjust the package names (e.g., `ros-foxy-turtlesim`).

You can copy the entire Markdown block above and paste it directly into your `README.md` file on GitHub. Let me know if you need further adjustments!
