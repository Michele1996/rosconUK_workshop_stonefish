"""BlueROV2 survey, collision/occlusion scene and autonomous inspection lab."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg=FindPackageShare('roscon_stonefish_workshop')
    stonefish=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('stonefish_ros2'),'launch','stonefish_simulator.launch.py'])),
        launch_arguments={
            'simulation_data':PathJoinSubstitution([pkg,'data']),
            'scenario_desc':PathJoinSubstitution([pkg,'data','scenarios',LaunchConfiguration('scenario')]),
            'simulation_rate':'100.0',
            'window_res_x':'1280','window_res_y':'720',
            'rendering_quality':'high',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value='bluerov2_photogrammetry.scn'),
        DeclareLaunchArgument('controller',default_value='false'),
        DeclareLaunchArgument('capture',default_value='false'),
        DeclareLaunchArgument('autonomy',default_value='false'),
        DeclareLaunchArgument('target',default_value='cat'),
        DeclareLaunchArgument('autonomy_mode',default_value='reference'),
        DeclareLaunchArgument('plan_file',default_value=''),
        stonefish,
        Node(package='roscon_stonefish_workshop',executable='bluerov_pid',
             name='photogrammetry_waypoint_pid',output='screen',
             parameters=[PathJoinSubstitution([pkg,'config','bluerov_pid.yaml'])],
             condition=IfCondition(LaunchConfiguration('controller'))),
        Node(package='roscon_stonefish_workshop',executable='photogrammetry_capture',
             name='photogrammetry_capture',output='screen',
             condition=IfCondition(LaunchConfiguration('capture'))),
        Node(package='roscon_stonefish_workshop',executable='inspection_reference',
             name='bluerov_inspection_reference',output='screen',
             parameters=[{'target':LaunchConfiguration('target'), 'mode':LaunchConfiguration('autonomy_mode'),
                          'plan_file':LaunchConfiguration('plan_file')}],
             condition=IfCondition(LaunchConfiguration('autonomy')))
    ])
