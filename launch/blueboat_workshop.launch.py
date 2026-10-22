"""ROSCon BlueBoat Grand Prix: Stonefish + independent judge + thruster bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('roscon_stonefish_workshop')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('stonefish_ros2'),'launch','stonefish_simulator.launch.py'
        ])),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([pkg,'data']),
            'scenario_desc': PathJoinSubstitution([pkg,'data','scenarios',
                                                   LaunchConfiguration('scenario')]),
            'simulation_rate':'100.0',
            'window_res_x':'1280',
            'window_res_y':'720',
            'rendering_quality':'high',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value='blueboat_race.scn'),
        DeclareLaunchArgument('team',default_value='Team 1'),
        DeclareLaunchArgument('start_pid',default_value='false',
                              description='Start sensor-only participant heading PID (surge=0).'),
        DeclareLaunchArgument('race_monitor',default_value='true'),
        DeclareLaunchArgument('results_dir',default_value='~/.ros/roscon_blueboat_grand_prix'),
        DeclareLaunchArgument('thruster_adapter',default_value='true'),
        sim,
        Node(package='roscon_stonefish_workshop',executable='blueboat_race_monitor',
             name='race_judge',output='screen',
             parameters=[{'course_file':PathJoinSubstitution([pkg,'config','blueboat_race.json']),
                          'team':LaunchConfiguration('team'),
                          'results_dir': LaunchConfiguration('results_dir')}],
             condition=IfCondition(LaunchConfiguration('race_monitor'))),
        Node(package='roscon_stonefish_workshop',executable='blueboat_thruster_adapter',
             name='blueboat_thrust_bridge',output='screen',
             condition=IfCondition(LaunchConfiguration('thruster_adapter'))),
        Node(package='roscon_stonefish_workshop',executable='blueboat_pid',
             name='participant_pid',output='screen',
             condition=IfCondition(LaunchConfiguration('start_pid'))),
    ])
