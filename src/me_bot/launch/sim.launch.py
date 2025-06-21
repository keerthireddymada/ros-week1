import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = os.path.join(os.getenv('HOME'), 'me_ws', 'src', 'me_bot')
    urdf_xacro_file = os.path.join(pkg_path, 'robot', 'me_bot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'me_world.sdf')
    controller_yaml = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')

    return LaunchDescription([
        # Start Gazebo properly with ROS clock support
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py'),
            launch_arguments={'world': world_file}.items()
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),

        # Robot State Publisher from xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': Command(['xacro ', urdf_xacro_file]),
                    'use_sim_time': True
                }
            ]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'me_bot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # Load Diff Drive Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            parameters=[controller_yaml],
            output='screen'
        ),

        # Teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/cmd_vel', '/input_cmd_vel')]
        ),

        # Obstacle Stopper Node
        Node(
            package='me_bot',
            executable='obstacle_stopper',
            name='obstacle_stopper',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Launch RViz 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'me_rviz.rviz')],
            output='screen'
        )
    ])
