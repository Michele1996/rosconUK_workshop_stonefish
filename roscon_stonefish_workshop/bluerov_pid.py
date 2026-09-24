#!/usr/bin/env python3
"""
Provided low-level BlueROV2 controller.

Participants should NOT need to implement the complete low-level controller.
They plan survey waypoints. The controller tracks x/y/depth/yaw references.

It publishes geometry_msgs/Twist so the vehicle-specific thruster adapter can
map desired body commands to the actual Stonefish/COLA2 thruster interface.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from .control_utils import PID, clamp, wrap_pi
from .blueboat_pid import yaw_from_quat

class BlueROVPID(Node):
    def __init__(self):
        super().__init__('bluerov_pid')
        self.declare_parameter('odom_topic', '/bluerov2/odom')
        self.declare_parameter('setpoint_topic', '/bluerov2/setpoint')
        self.declare_parameter('cmd_topic', '/bluerov2/cmd_vel')

        self.x_pid = PID(0.45, 0.0, 0.08, limit=0.6)
        self.y_pid = PID(0.45, 0.0, 0.08, limit=0.6)
        self.z_pid = PID(0.65, 0.02, 0.12, limit=0.7)
        self.yaw_pid = PID(1.20, 0.0, 0.12, limit=0.8)
        self.target = None
        self.last_t = None

        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.create_subscription(PoseStamped, self.get_parameter('setpoint_topic').value,
                                 self.target_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.odom_cb, 10)

    def target_cb(self, msg):
        self.target = msg.pose

    def odom_cb(self, msg):
        if self.target is None:
            return
        now = self.get_clock().now().nanoseconds*1e-9
        dt = 0.02 if self.last_t is None else max(1e-3, now-self.last_t)
        self.last_t = now

        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        tyaw = yaw_from_quat(self.target.orientation)

        # World-frame position controller.
        ux = self.x_pid.step(self.target.position.x-p.x, dt)
        uy = self.y_pid.step(self.target.position.y-p.y, dt)
        uz = self.z_pid.step(self.target.position.z-p.z, dt)

        # Rotate horizontal command into vehicle body frame.
        surge =  math.cos(yaw)*ux + math.sin(yaw)*uy
        sway  = -math.sin(yaw)*ux + math.cos(yaw)*uy

        out = Twist()
        out.linear.x = surge
        out.linear.y = sway
        out.linear.z = uz
        out.angular.z = self.yaw_pid.step(wrap_pi(tyaw-yaw), dt)
        self.pub.publish(out)

def main():
    rclpy.init()
    n = BlueROVPID()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
