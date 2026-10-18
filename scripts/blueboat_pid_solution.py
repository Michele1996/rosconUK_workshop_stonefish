#!/usr/bin/env python3
"""Instructor example: PID waypoint/heading tracking for ROSCon BlueBoat race.

Run after sourcing workspace: python3 scripts/blueboat_pid_solution.py
Reference starting point, NOT claimed to be tuned/validated fastest in Stonefish.
"""
import math
import rclpy

from roscon_stonefish_workshop.blueboat_pid import BlueBoatPID
from roscon_stonefish_workshop.control_utils import wrap_pi
from roscon_stonefish_workshop.race_geometry import nearest_centerline, sample_course, length


class InstructorPID(BlueBoatPID):
    def compute_command(self,x,y,yaw,dt):
        lateral,s = nearest_centerline(self.course,(x,y))
        lookahead = self.get_parameter('lookahead_m').value
        target,_dir,_side = sample_course(self.course,s+lookahead)
        dx,dy = target[0]-x,target[1]-y
        heading_error = wrap_pi(math.atan2(dy,dx)-yaw)
        distance = math.hypot(dx,dy)
        if s >= length(self.course)-0.5 and distance < 0.8:
            return 0.0,0.0
        # PID speed and heading; slow down when pointing away from direction.
        surge = self.distance_pid.step(distance,dt)
        surge *= max(0.0,math.cos(heading_error))
        if abs(heading_error)>0.65:
            surge *= 0.5
        yaw_command = self.heading_pid.step(heading_error,dt)
        return surge,yaw_command


def main():
    rclpy.init()
    node = InstructorPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
