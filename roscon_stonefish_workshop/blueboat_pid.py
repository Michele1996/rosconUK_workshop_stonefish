#!/usr/bin/env python3
"""BlueBoat participant example: sensor-only heading PID (NO race file access).

Uses /blueboat/odom orientation to hold a chosen heading. The default forward
speed is ZERO, so participants control when to enable motion. Not a solved race.

FAIR PLAY: no JSON/CSV/YAML course, waypoint lists or judge telemetry as input.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .control_utils import PID, clamp, wrap_pi
from .fair_play import FairPlayViolation, guard_participant_route_parameters, reject_privileged_topic


def yaw_from_quat(q):
    sin_yaw = 2.0 * (q.w*q.z + q.x*q.y)
    cos_yaw = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(sin_yaw, cos_yaw)


class BlueBoatPID(Node):
    def __init__(self):
        super().__init__('blueboat_pid')
        guard_participant_route_parameters(self)
        self.declare_parameter('odom_topic', '/blueboat/odom')
        self.declare_parameter('cmd_topic', '/blueboat/cmd_vel')
        self.declare_parameter('target_heading_rad', 0.0)
        self.declare_parameter('surge_command', 0.0)
        self.declare_parameter('max_surge', 0.7)
        reject_privileged_topic(self.get_parameter('odom_topic').value)
        # TODO(participant): tune the yaw PID after checking positive yaw direction.
        self.heading_pid = PID(kp=1.0, ki=0.0, kd=0.08, limit=0.55)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.subscription = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.last_t = None
        self.get_logger().info(
            'Sensor-only heading PID ready: surge_command=0 by default. '
            'FAIR PLAY: precomputed race maps/waypoints are prohibited.')

    def compute_command(self, x, y, yaw, dt):
        # x,y kept in the API for participant extensions using permitted odom.
        heading = float(self.get_parameter('target_heading_rad').value)
        error = wrap_pi(heading - yaw)
        yaw_command = self.heading_pid.step(error, dt)
        # TODO(participant): select speed and target heading from LIVE observations.
        # Do NOT load the hidden course geometry to choose a heading.
        max_surge = max(0.0, abs(float(self.get_parameter('max_surge').value)))
        surge = clamp(float(self.get_parameter('surge_command').value),
                      -max_surge, max_surge)
        return surge, yaw_command

    def odom_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        dt = 0.05 if self.last_t is None or stamp <= self.last_t else clamp(
            stamp-self.last_t, 0.01, 0.3)
        if self.last_t is not None and stamp < self.last_t-0.1:
            self.heading_pid.reset()
        self.last_t = stamp
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        surge, turn = self.compute_command(p.x, p.y, yaw, dt)
        out = Twist()
        out.linear.x = float(surge)
        out.angular.z = float(turn)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BlueBoatPID()
        rclpy.spin(node)
    except FairPlayViolation:
        raise SystemExit(2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
