from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(           #publisher_node
            package='my_node',
            executable='publisher',  
            name='number_publisher',
            output='screen'
        ),
        Node(           #subsciber_node
            package='my_node',
            executable='subscriber',  
            name='number_subscriber',
            output='screen'
        )
    ])
