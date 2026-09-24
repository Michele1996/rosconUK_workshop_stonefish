#!/usr/bin/env python3
"""
ROSCon exercise: survey planner.

Complete make_survey() so that it returns a sequence of PoseStamped waypoints
that gives the camera sufficient overlap over the target.

The final target bounds will be updated when the final underwater environment
is added.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

def pose(x, y, z, yaw):
    p = PoseStamped()
    p.header.frame_id = 'world'
    p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
    q = quaternion_from_euler(0.0, 0.0, yaw)
    p.pose.orientation.x, p.pose.orientation.y = q[0], q[1]
    p.pose.orientation.z, p.pose.orientation.w = q[2], q[3]
    return p

def make_survey():
    # Example reference pattern. Update bounds for the final environment.
    xmin, xmax = 0.0, 8.0
    y0, line_spacing, lines = -3.0, 1.0, 6
    z = -2.0
    waypoints = []
    for row in range(lines):
        y = y0 + row*line_spacing
        if row % 2 == 0:
            waypoints += [pose(xmin, y, z, 0.0), pose(xmax, y, z, 0.0)]
        else:
            waypoints += [pose(xmax, y, z, math.pi), pose(xmin, y, z, math.pi)]
    return waypoints

class SurveyPlanner(Node):
    def __init__(self):
        super().__init__('survey_planner')
        self.pub = self.create_publisher(PoseStamped, '/bluerov2/setpoint', 10)
        self.waypoints = make_survey()
        self.i = 0
        self.create_timer(4.0, self.send_next)

    def send_next(self):
        if self.i >= len(self.waypoints):
            return
        p = self.waypoints[self.i]
        p.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(p)
        self.get_logger().info(f'Waypoint {self.i+1}/{len(self.waypoints)}')
        self.i += 1

def main():
    rclpy.init()
    n = SurveyPlanner()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
