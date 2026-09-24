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
    # TODO(participant):
    # Return a lawn-mower/inspection path.
    #
    # Think about:
    #   - camera overlap
    #   - stand-off distance
    #   - line spacing
    #   - heading
    #   - avoiding unnecessary turns
    #
    # Example only:
    waypoints = [
        pose(0.0, 0.0, -2.0, 0.0),
    ]
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
