#!/usr/bin/env python3
"""Convert body commands into Stonefish BlueROV2 thruster setpoints.

Grounded in the workshop's existing joystick controller:
  topic: /bluerov/controller/thruster_setpoints_sim
  type:  std_msgs/msg/Float64MultiArray
  command order: [forward, side, rotation, depth]

The same 8x4 allocation matrix is used here.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

ALLOCATION = np.array([
    [ 1,  1,  1,  0],
    [ 1, -1, -1,  0],
    [-1,  1, -1,  0],
    [-1, -1,  1,  0],
    [ 0,  0,  0,  1],
    [ 0,  0,  0, -1],
    [ 0,  0,  0, -1],
    [ 0,  0,  0,  1],
], dtype=float)

class BlueROVThrusterAdapter(Node):
    def __init__(self):
        super().__init__('bluerov_thruster_adapter')
        self.declare_parameter('cmd_topic', '/bluerov2/cmd_vel')
        self.declare_parameter(
            'thruster_topic', '/bluerov/controller/thruster_setpoints_sim')
        self.pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('thruster_topic').value, 10)
        self.create_subscription(
            Twist, self.get_parameter('cmd_topic').value, self.cb, 10)

    def cb(self, msg):
        # [surge/forward, sway/side, yaw/rotation, heave/depth]
        command = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            msg.linear.z,
        ], dtype=float)

        thrusters = ALLOCATION @ command

        # Preserve the normalization behaviour of the existing joystick node.
        maximum = float(np.max(np.abs(thrusters)))
        if maximum > 1.0:
            thrusters /= maximum

        out = Float64MultiArray()
        out.data = thrusters.tolist()
        self.pub.publish(out)

def main():
    rclpy.init()
    node = BlueROVThrusterAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
