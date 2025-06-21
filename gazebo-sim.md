# ME Bot 

**ME Bot** is a basic differential drive robot simulation built using ROS 2 Humble, Gazebo Classic, and RViz. It features a robot model with camera and LIDAR sensors, and supports keyboard teleoperation. 

---

## Project Directory Structure (Essential Only)

ME_WS/

├── src/

│ └── me_bot/

│ ├── launch/

│ │ └── sim.launch.py

│ ├── config/

│ │ └── diff_drive_controller.yaml 

│ ├── robot/

│ │ └── me_bot.urdf.xacro 

│ ├── rviz/

│ │ └── me_rviz.rviz # RViz saved configuration

│ ├── worlds/

│ │ └── me_world.sdf

│ ├── me_bot/

│ │ └── obstacle_stopper.py 

│ ├── package.xml

│ └── setup.py 

---

## Features Implemented

### 1. Robot Description (URDF + Xacro)
- Base chassis (lavender box)
- 4 wheels using Xacro macro
  - Material: `black`
  - Properly positioned using correct `xyz` in `<joint>` origin
- Camera + LIDAR sensors
  - Mounted on top with fixed joints
  - Material: `black`
- Visualized in Gazebo and RViz

### 2. Controllers
- Differential drive using `diff_drive_controller`
- Configured wheel separation, radius, limits, odom frame, etc.

### 3. Sensors
- **LIDAR**: 360° laser scan via `libgazebo_ros_ray_sensor.so`
- **Camera**: RGB image stream via `libgazebo_ros_camera.so`

### 4. Gazebo Integration
- Used `gazebo_ros` plugins
- Spawned robot using `spawn_entity.py`
- Included `libgazebo_ros_diff_drive.so` for motion

### 5. RViz Integration
- Visualized robot in RViz and saved config: `me_rviz.rviz`
- Saved via:  
  `File → Save Config As → me_rviz.rviz`

### 6. Launch File
File: `me_bot/launch/sim.launch.py`

- Launches Gazebo with world
- Publishes robot state and joints
- Spawns robot from `robot_description`
- Spawns `diff_drive_controller`
- Launches custom `obstacle_stopper` node
- Runs Rviz

---

## How to Run 

Follow these simple steps to clone, build, and run the `me_bot` simulation package with Gazebo and RViz.

### 1. Clone the Repository

First, open a terminal and create a workspace:

```bash
mkdir -p ~/me_ws/src
cd ~/me_ws/src
# Clone your package here (replace with my repo URL)
git clone <my-repo-url>
```

### 3. Build the Workspace

Now build your workspace:

```
cd ~/me_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

After successful build:

```
source install/setup.bash
```

### 4. Launch the Simulation

Now launch the robot in Gazebo with RViz:

```
ros2 launch me_bot sim.launch.py
```

This will:

    Start Gazebo with custom world and robot

    Start RViz with saved configuration

    Spawn the robot and load the diff drive controller

    Launch the obstacle_stopper node

### 5. Control the Robot (Teleop)

Open a new terminal, source the workspace, and run:

```
source ~/me_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/input_cmd_vel
```

Use the keyboard to move the robot around with the controls mentioned there.

---
