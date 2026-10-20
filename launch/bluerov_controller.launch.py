"""Single-publisher BlueROV2 waypoint PID for an already running scene."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(package='roscon_stonefish_workshop', executable='bluerov_pid',
             name='photogrammetry_waypoint_pid', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('roscon_stonefish_workshop'),
                                               'config', 'bluerov_pid.yaml'])])
    ])
