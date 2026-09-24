# Exercise 2 — BlueROV2 Control

The BlueROV2 low-level controller is provided. Start the control stack with:

```bash
ros2 launch roscon_stonefish_workshop bluerov_controller.launch.py
```

The flow is:

```text
PoseStamped setpoint
        ↓
x/y/depth/yaw PID
        ↓
geometry_msgs/Twist
        ↓
8×4 thruster allocation
        ↓
Float64MultiArray
        ↓
/bluerov/controller/thruster_setpoints_sim
```

The allocation maps `[forward, side, rotation, depth]` into eight thrusters.
Commands are normalized whenever an allocated thruster magnitude exceeds 1.0.

Participants are not expected to reimplement this low-level controller. Inspect
it, experiment with setpoints, then move to the survey-planning challenge.

The final simulator environment/launch will be added when the underwater survey
scene is supplied.
