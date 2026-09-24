from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    simulator = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('stonefish_ros2'),
            'launch',
            'stonefish_simulator.launch.py'
        ]),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([
                FindPackageShare('roscon_stonefish_workshop'), 'data'
            ]),
            'scenario_desc': PathJoinSubstitution([
                FindPackageShare('roscon_stonefish_workshop'),
                'data', 'scenarios', 'blueboat_goal.scn'
            ]),
            'simulation_rate': '100.0',
            'window_res_x': '1280',
            'window_res_y': '720',
            'rendering_quality': 'high',
        }.items()
    )
    return LaunchDescription([simulator])
