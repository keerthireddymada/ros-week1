This repository covers what I’ve learned about ROS2 in this week—how to download it, how it works, key concepts, and basic usage. I've included core concepts like nodes, topics, services, and parameters, along with code for nodes to demonstrate how they function in different ways. The focus is on understanding the basics of ROS2, from installation and setup to writing and running simple programs. I tried to summarize my understanding for quick reference. 

---

# ROS2
ROS 2 (Robot Operating System 2) is a set of common libraries and tools on which we build more complex robotic systems. I tried to cover the core concepts, set up a simple workspace, explore the turtlesim simulator, and create basic Python nodes to understand how ROS 2 works. 

## ROS2 Humble
ROS2 Humble is a release of ROS2 which is optimized for Ubuntu 22.04 (Jammy Jellyfish) and is widely used for stable robotic development.

### Installing ROS2 Humble 

Step 1: Set Locale & Enable Universe Repo

```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
```

Step 2: Add ROS 2 GPG Key & Repository

```
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Step 3: Install ROS 2 Humble

```
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop  # Full install (ROS, Rviz, GUI tools)
```


Step 4: Source ROS 2 in Your Shell

```
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Key Concepts in ROS 2

#### Nodes
Smaller programs in ROS which talk to one another and run simultaneously. Each node is assigned a specific task. 

To run a node:

```
ros2 run <package_name> <node_name>
```

To see all running nodes:

```
ros2 node list
```

#### Topics
Topic is a location where node publishes message. 
Nodes which publish messages into topic are publisher nodes. Nodes which subscribe to topics to receive messages are subscriber nodes.
We need to specify the name of the topic and the type of message that will be published into it.

To see all available topics:
```
ros2 topic list
```

To view messages in a topic (like a subscriber):
```
ros2 topic echo <topic_name>
```

#### Services
Services are like a exchange between two nodes. One nodes sends a message and waits for the other node to reply to it.
The difference between topics and services is that in topics, publisher nodes publishes and whichever node has anything to do with - it would subscribe to that topic but thats not the case in services, the first node would wait for reply.

To see all available services:
```
ros2 service list
```

To call a service manually:
```
ros2 service call <service_name> <service_message>
```

#### Actions
Actions are a way of communication in ROS2 for long running tasks consisting of goal, feedback, result. They act mostly like services except that they can get cancelled and they also provide feedback
We have Action Server and Action Client

![Actions](https://github.com/keerthireddymada/ros-week1/blob/main/Screenshot%20from%202025-05-17%2020-52-34.png)

#### Parameters
Parameters are like settings for a node that you can change while the node is running. Like adjusting a robot’s speed without stopping it. 

To change parameters when running a node:
```
--ros-args -p
```

#### Remapping
Remapping lets you change the name of a topic. This is helpful when two nodes need to communicate but their topic names don’t match. We will use it to avoid conflicts or connect nodes that use different topic names by renaming topics when you run the node so they can become suitable publisher or subscriber nodes through a single topic.

To remap a topic when running a node:
```
--ros-args -r
```

#### Launching files
Launching is a scripting system called that lets us configure and launch a bunch of nodes together in a group. 

To run a launch file from a package:
```
ros2 launch <package_name> <launch_file_name>
```

#### Packages
A package is a container of a variety of things, such as a single node, a set of nodes , configuration files, etc so that we can reuse common code in different projects.

## Commands for ROS 2 Concepts
Here are the key commands to work with nodes, topics, services, parameters, and remapping:

### Setting Up a Workspace
Workspace will contain our ROS packages.

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
A package is like a container for nodes and other ROS 2 files. It helps keep our project organized.

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
__init_.py is added because python then recognizes it as a package and lets us use modules in it.

## Exploring Turtlesim

### What is Turtlesim?
Turtlesim is a simple simulator in ROS 2 that shows a turtle on a blue screen. Whatever we wish our robot to do, we can just learn by making the turtle do them.

### Installing and Running Turtlesim
First, you need to install turtlesim and run it to see the turtle on the screen. You can then use keyboard controls to move the turtle around.

### Using Services to Teleport the Turtle
the following command would let us teleport the turtle for absolute coordinates:
```
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: <x> , y: <y> , theta: 0.0}"
```

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

# Echo turtle's pose
ros2 topic echo /turtle1/pose           #(continuously prints the turtle's x, y, theta (angle), velocity)

# Echo Turtle's Commanded Velocity
ros2 topic echo /turtle1/cmd_vel         #(shows linear.x (forward/back) and angular.z (rotation) commands sent to the turtle)

# Teleport the turtle to position (3, 3) with a 90-degree angle
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 3.0, y: 3.0, theta: 1.57}"
```

### Visualizing the Turtle’s Movement

For the above teleport service command, we would see the turtlesim like below:

![Turtle teleporting](https://github.com/keerthireddymada/ros-week1/blob/main/Screenshot%20from%202025-05-18%2000-51-07.png)

## Python Nodes
I made two packages in `my_ws` which are `my_node` and `turtle_controller`. 

#### my_node
It consists of a publisher node `hello_pub.py' which would publish a hello message and a subscriber node 'hello_sub.py' which would receive the hello message which I did first to understand the working of the publisher and subscriber nodes.

After that, it also consists the publisher node `publisher_node' which would send number and subscriber node `subscriber_node` which would receive it and calculate its square give it as output.
### Creating Publisher and Subscriber Nodes
Creating two nodes: a publisher that sends a number and a subscriber that receives it and calculates its square.

#### Publisher Node (`publisher_node.py`)
This node sends number 12 to a topic called `number` every second. It’s like a worker who keeps shouting a number to anyone listening.

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
   - `self.create_timer(timer_period, self.publish_number)`: Calls the `publish_number` function every timer_period seconds.
   - `rclpy.spin(node)`: Keeps the node running so the timer works.
   - `node.destroy_node()` and `rclpy.shutdown()`: Clean up when the node stops.
   - `self.publisher.publish(msg)`: Sends the `msg` to the topic associated with self.publisher which in this case is 'number' and the nodes subscribed to that topic will receive it.
   - `rclpy.shutdown()`: Cleans up and shuts down.

    

#### Subscriber Node (`subscriber_node.py`)
This node listens to the `number` topic, calculates the square of the number, and prints it. It’s like a worker who listens to the shouted number and squares it and gives it.

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
But before this, we need to go to `setup.py` and do node registration under console scripts.
```
'console_scripts': [
            'publisher = my_node.publisher_node:main',
            'subscriber = my_node.subscriber_node:main',
            'hello_publisher = my_node.hello_pub:main',
            'hello_subscriber = my_node.hello_sub:main',
        ],
```

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

### turtle_controller
In this package, we made a publisher node `turtle_square.py` that makes the turtle in turtlesim move in a square of side 2. If we run it individually, it will make the turtle move in the square.
In the same package, I made a subscriber node `pose_subscriber.py` which will constantly give the pose(x,y,theta) of the turtle as it traverses across the square because of publisher node.
   
- **What does this code do?**
   - `self.create_publisher(Twist, '/turtle1/cmd_vel', 10)`: Publishes `Twist` messages to control the turtle’s movement.
   - `self.create_timer(0.1, self.move_square)`: Calls `move_square` every 0.1 seconds.
   - The node alternates between moving forward (`FORWARD` state) and turning (`TURN` state) to draw a square with a side length of 2.0 units.
   - After completing four sides, it logs “Completed one square!” and starts over.

![Turtle square](https://github.com/keerthireddymada/ros-week1/blob/main/Screenshot%20from%202025-05-18%2005-27-20.png)
#### Commands to Run the Turtle Controller
First we should add dependencies in package.xml
```
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>turtlesim</exec_depend>
```
Then we should register nodes under console_scripts in setup.py
```
'console_scripts': [
            'square_movement = turtle_controller.turtle_square:main',
            'pose_subscriber = turtle_controller.pose_subscriber:main',
        ],
```
```
# Build and source the workspace
cd ~/my_ws
colcon build
source install/setup.bash

# Run turtlesim
ros2 run turtlesim turtlesim_node(in one terminal)

# Run the turtle controller 
ros2 run turtle_controller square_movement (in second terminal)
ros2 run turtle_controller pose_subscriber (in third terminal)
```

## Resources that I used:
[Articulated Robotics ROS Overview](https://articulatedrobotics.xyz/tutorials/ready-for-ros/ros-overview).

[For understanding theory concepts](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

[For understanding publisher and subscriber nodes](https://www.youtube.com/watch?v=VDotKYuKcVY&list=PLO89phzZmnHi5GCama8rS0kg3jcEXTq7I&index=4)

[For basic understanding of actions](https://www.youtube.com/watch?v=wKC1znfJ4oM)



---




