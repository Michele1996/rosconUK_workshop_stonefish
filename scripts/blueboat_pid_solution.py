#!/usr/bin/env python3
"""
ROSCon exercise: BlueBoat PID controller.

Student task:
  1. Tune the distance and heading PID gains.
  2. Complete compute_command().
  3. Drive the surface vehicle to the red goal cube.

IMPORTANT:
The workshop package does not assume a fabricated Stonefish message type.
This node consumes standard nav_msgs/Odometry and publishes geometry_msgs/Twist.
The adapter from Twist to the scenario's actual thruster-setpoint message is
kept separate (thruster_adapter.py) and must be configured for the tested
Stonefish/COLA2 interface before the workshop.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .control_utils import PID, clamp, wrap_pi

def yaw_from_quat(q):
    siny = 2.0*(q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)

class BlueBoatPID(Node):
    def __init__(self):
        super().__init__('blueboat_pid')
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('odom_topic', '/blueboat/odom')
        self.declare_parameter('cmd_topic', '/blueboat/cmd_vel')

        # TODO(participant): tune these during Exercise 1.
        self.distance_pid = PID(kp=0.35, ki=0.0, kd=0.05, limit=0.7)
        self.heading_pid  = PID(kp=1.2,  ki=0.0, kd=0.10, limit=0.8)

        self.pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.last_t = None

    def compute_command(self, x, y, yaw, dt):
        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)
        dx, dy = gx-x, gy-y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = wrap_pi(desired_yaw-yaw)

        if distance < 0.5:
            return 0.0, 0.0, distance
        surge = self.distance_pid.step(distance, dt)
        # Do not charge forward when badly misaligned.
        surge *= max(0.0, math.cos(heading_error))
        yaw_rate = self.heading_pid.step(heading_error, dt)
        return surge, yaw_rate, distance

    def odom_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.02 if self.last_t is None else max(1e-3, now-self.last_t)
        self.last_t = now
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        surge, yaw_rate, distance = self.compute_command(p.x, p.y, yaw, dt)
        out = Twist()
        out.linear.x = float(surge)
        out.angular.z = float(yaw_rate)
        self.pub.publish(out)
        if distance < 0.5:
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)

def main():
    rclpy.init()
    n = BlueBoatPID()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
