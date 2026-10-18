#!/usr/bin/env python3
"""BlueBoat Grand Prix starter: implement PID surge and tune heading.

Read-only official timing uses /blueboat/race/ground_truth_odom.
Students navigate with /blueboat/odom (same source in the introductory race).
They may instead use a learned policy and publish the same /blueboat/cmd_vel.
"""
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .control_utils import PID, wrap_pi
from .race_geometry import load_course, nearest_centerline, sample_course


def yaw_from_quat(q):
    siny = 2.0*(q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)


class BlueBoatPID(Node):
    def __init__(self):
        super().__init__('blueboat_pid')
        default_course = str(Path(get_package_share_directory(
            'roscon_stonefish_workshop')) / 'config' / 'blueboat_race.json')
        self.declare_parameter('course_file', default_course)
        self.declare_parameter('odom_topic', '/blueboat/odom')
        self.declare_parameter('cmd_topic', '/blueboat/cmd_vel')
        self.declare_parameter('lookahead_m', 3.5)
        self.course = load_course(self.get_parameter('course_file').value)
        # TODO(participant): tune gains to finish fast, without track violations.
        self.distance_pid = PID(kp=0.4, ki=0.0, kd=0.05, limit=0.7)
        self.heading_pid = PID(kp=1.0, ki=0.0, kd=0.08, limit=0.55)
        self.pub = self.create_publisher(Twist,self.get_parameter('cmd_topic').value,10)
        self.create_subscription(Odometry,self.get_parameter('odom_topic').value,
                                 self.odom_cb,10)
        self.last_t = None

    def compute_command(self, x, y, yaw, dt):
        _dist, s = nearest_centerline(self.course, (x,y))
        target, _forward, _left = sample_course(
            self.course, s+self.get_parameter('lookahead_m').value)
        dx, dy = target[0]-x, target[1]-y
        distance = math.hypot(dx,dy)
        error = wrap_pi(math.atan2(dy,dx)-yaw)
        # TODO(participant): replace the zero surge with distance PID and
        # slow down on corners, otherwise the boat will never race.
        surge = 0.0
        yaw_command = self.heading_pid.step(error,dt)
        return surge,yaw_command

    def odom_cb(self, msg):
        timestamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        dt = 0.05 if self.last_t is None or timestamp <= self.last_t else max(0.01,min(0.3,timestamp-self.last_t))
        if self.last_t is not None and timestamp < self.last_t-0.1:
            self.distance_pid.reset()
            self.heading_pid.reset()
        self.last_t = timestamp
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        surge, yaw_command = self.compute_command(p.x,p.y,yaw,dt)
        output = Twist()
        output.linear.x = float(surge)
        output.angular.z = float(yaw_command)
        self.pub.publish(output)


def main():
    rclpy.init()
    node = BlueBoatPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
