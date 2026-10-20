#!/usr/bin/env python3
"""Simple, heading-first BlueROV2 waypoint PID for Stonefish NED.

The controller intentionally does NOT use odometry Twist: some simulator
configurations report twist in a different frame. Pose differences provide
body-forward speed, vertical velocity, and yaw rate for simple D damping.

Modes:
  TURN   -> rotate toward the next XY waypoint, hold current depth.
  MOVE   -> travel forward toward the waypoint, control its depth.
  SETTLE -> stop translation and turn toward the requested inspection yaw.

One publisher commands all eight thrusters using the workshop joystick matrix.
Yaw/heave physical setpoint conventions depend on the uploaded Stonefish model;
change yaw_sign/heave_sign in config if the direct-thruster test shows reverse.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String



from .bluerov_waypoint_math import (
    WaypointGains, waypoint_efforts, allocate, clamp, wrap_pi,
    yaw_from_quat, body_forward_velocity)

class BlueROVWaypointPID(Node):
    def __init__(self):
        super().__init__('bluerov_pid')
        defaults = {
            'odom_topic':'/bluerov2/odometry',
            'setpoint_topic':'/bluerov2/setpoint',
            'thruster_topic':'/bluerov/controller/thruster_setpoints_sim',
            'debug_topic':'/bluerov2/waypoint_pid/status',
            'max_thruster_setpoint':0.65,
            'setpoint_stale_timeout_s':5.0,
            'odom_stale_timeout_s':2.0,
            'xy_tolerance':0.65,
            'turn_threshold_rad':0.52,
            'kp_forward':0.28, 'kd_forward':0.13, 'max_forward':0.58,
            'kp_yaw':0.3, 'kd_yaw':0.14, 'max_yaw':0.34,
            'kp_depth':0.16, 'ki_depth':0.008, 'kd_depth':0.2,
            'max_heave':0.2, 'depth_deadband':0.06,
            'yaw_sign':-1.0, 'heave_sign':-1.0,
        }
        for key, val in defaults.items():
            self.declare_parameter(key, val)
        self.target = None
        self.target_time = None
        self.odom = None
        self.odom_time = None
        self.last_pose = None
        self.last_time = None
        self.speed_forward = 0.0
        self.speed_down = 0.0
        self.rate_yaw = 0.0
        self.depth_integral = 0.0
        self.mode = 'WAIT'
        self.turn_depth = None
        self.debug_last = 0.0
        self.pub = self.create_publisher(Float64MultiArray, self.p('thruster_topic'), 10)
        self.debug_pub = self.create_publisher(String, self.p('debug_topic'), 10)
        self.create_subscription(PoseStamped, self.p('setpoint_topic'), self.on_setpoint, 10)
        self.create_subscription(Odometry, self.p('odom_topic'), self.on_odom, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Simple heading-first waypoint PID, 8x4 Logitech allocation, direct thrusters. '
                               'Initial yaw_sign=%.0f, heave_sign=%.0f; verify direction in Stonefish.' %
                               (self.p('yaw_sign'), self.p('heave_sign')))

    def p(self, name):
        return self.get_parameter(name).value

    def gains(self):
        return WaypointGains(**{k: float(self.p(k)) for k in WaypointGains.__dataclass_fields__})

    def on_setpoint(self, msg):
        # Do not reset damping/integral every time the mission republishes the
        # same goal at 2 Hz. Only clear accumulated depth bias for a new goal.
        p = msg.pose.position
        q = msg.pose.orientation
        value = (float(p.x),float(p.y),float(p.z),float(yaw_from_quat(q)))
        if self.target is None or (math.dist(value[:3],self.target[:3]) > 0.15):
            self.depth_integral = 0.0
            self.turn_depth = None
            self.mode = 'WAIT'
        self.target = value
        self.target_time = time.monotonic()

    def on_odom(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()

    def send(self, commands):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in commands]
        self.pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if self.odom is None:
            self.send([0.0]*8)
            return
        pose = self.odom.pose.pose
        cur = (float(pose.position.x),float(pose.position.y),
               float(pose.position.z),float(yaw_from_quat(pose.orientation)))
        if self.last_pose is not None and self.last_time is not None:
            dt = now-self.last_time
            if 0.01 < dt < 0.3:
                vx = (cur[0]-self.last_pose[0])/dt
                vy = (cur[1]-self.last_pose[1])/dt
                down = (cur[2]-self.last_pose[2])/dt
                r = wrap_pi(cur[3]-self.last_pose[3])/dt
                # Low-pass finite-difference velocities instead of relying on
                # Odometry.twist being body- or world-frame.
                a = 0.22
                self.speed_forward = (1-a)*self.speed_forward + a*body_forward_velocity(vx,vy,cur[3])
                self.speed_down = (1-a)*self.speed_down + a*down
                self.rate_yaw = (1-a)*self.rate_yaw + a*r
        self.last_pose,self.last_time = cur,now
        if (self.target is None or self.target_time is None or
            now-self.target_time > float(self.p('setpoint_stale_timeout_s')) or
            self.odom_time is None or now-self.odom_time > float(self.p('odom_stale_timeout_s'))):
            self.mode = 'WAIT'
            self.depth_integral = 0.0
            self.send([0.0]*8)
            return
        gx,gy,gz,gyaw = self.target
        exy = math.hypot(gx-cur[0],gy-cur[1])
        bearing = math.atan2(gy-cur[1],gx-cur[0]) if exy > float(self.p('xy_tolerance')) else gyaw
        should_turn = exy > float(self.p('xy_tolerance')) and abs(wrap_pi(bearing-cur[3])) > float(self.p('turn_threshold_rad'))
        if should_turn and self.mode != 'TURN':
            self.turn_depth = cur[2]
        if not should_turn:
            self.turn_depth = None
        # Anti-windup: only integrate small depth errors when output is not saturated.
        if not should_turn and abs(gz-cur[2]) < 1.5:
            self.depth_integral = clamp(self.depth_integral + (gz-cur[2])*0.05,-0.45,0.45)
        commands, mode, dxy, ez, eyaw = waypoint_efforts(
            *cur, gx,gy,gz,gyaw,
            forward_speed=self.speed_forward,down_speed=self.speed_down,
            yaw_rate=self.rate_yaw, depth_integral=self.depth_integral,
            turn_depth=self.turn_depth,gains=self.gains())
        self.mode = mode
        motors = allocate(*commands,limit=float(self.p('max_thruster_setpoint')))
        self.send(motors)
        if now-self.debug_last >= 1.0:
            status = String()
            status.data = ('mode=%s d_xy=%.2fm depth_error=%.2fm yaw_error=%.1fdeg '
                           'surge=%.3f yaw=%.3f heave=%.3f depth=%.2fm' %
                           (mode,dxy,ez,math.degrees(eyaw),commands[0],commands[2],commands[3],cur[2]))
            self.debug_pub.publish(status)
            self.debug_last = now


def main():
    rclpy.init()
    node = BlueROVWaypointPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send([0.0]*8)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
