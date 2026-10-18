#!/usr/bin/env python3
"""Convert competition controller Twist to the 2 native Stonefish thrusters.

Order in blueboat.scn is [Thruster_R, Thruster_L]. All setpoints are [-1,1].
Zero output after a command timeout avoids uncontrolled propulsion after a node dies.
Convention: Stonefish world NED, positive yaw is towards starboard (+y).
Motor definitions use the known-working BlueROV inverted_setpoint=true setting.
The bridge negates mixer outputs to preserve the prior non-inverted sign convention.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def mix_thrusters(surge, yaw, yaw_sign=1.0):
    surge = max(-1.0, min(1.0, float(surge)))
    yaw = max(-1.0, min(1.0, float(yaw))) * float(yaw_sign)
    right = surge - yaw
    left = surge + yaw
    norm = max(1.0, abs(right), abs(left))
    return [right/norm, left/norm]


class BlueBoatThrusterAdapter(Node):
    def __init__(self):
        super().__init__('blueboat_thruster_adapter')
        self.declare_parameter('cmd_topic', '/blueboat/cmd_vel')
        self.declare_parameter('thruster_topic', '/blueboat/controller/thruster_setpoints_sim')
        self.declare_parameter('timeout_s', 2.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.pub = self.create_publisher(Float64MultiArray,
            self.get_parameter('thruster_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('cmd_topic').value,
                                 self.receive, 10)
        self.command = (0.0, 0.0)
        self.last_cmd = None
        self.create_timer(0.05, self.publish)
        self.get_logger().info('BlueBoat thruster bridge active [right, left].')

    def receive(self, msg):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().warn('Ignoring nonfinite controller command')
            return
        self.command = (msg.linear.x, msg.angular.z)
        self.last_cmd = self.get_clock().now().nanoseconds*1e-9

    def publish(self):
        now = self.get_clock().now().nanoseconds*1e-9
        timed_out = (self.last_cmd is None or
                     now-self.last_cmd > self.get_parameter('timeout_s').value)
        surge, yaw = (0.0, 0.0) if timed_out else self.command
        out = Float64MultiArray()
        out.data = [-x for x in mix_thrusters(surge,yaw,self.get_parameter('yaw_sign').value)]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = BlueBoatThrusterAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
