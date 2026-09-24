from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=(
            "The final BlueROV2 survey environment will be added before the "
            "workshop. The PID/controller/planner code is already available."
        ))
    ])
