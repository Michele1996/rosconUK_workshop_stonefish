from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roscon_stonefish_workshop',
            executable='bluerov_pid',
            output='screen',
        ),
        Node(
            package='roscon_stonefish_workshop',
            executable='thruster_adapter',
            output='screen',
        ),
    ])
