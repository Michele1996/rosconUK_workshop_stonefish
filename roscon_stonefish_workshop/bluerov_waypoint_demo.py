#!/usr/bin/env python3
"""Send a safe relative waypoint for debugging the BlueROV2 waypoint PID.

Example (with controller:=true, autonomy:=false):
  ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --forward 2 --seconds 45
  ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --down 0.4 --seconds 30
  ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --turn-deg 30 --seconds 25

One test at a time. The demo republishes the target; it never publishes thrusters.
"""
import argparse
import math
import time


def relative_goal(x, y, z, yaw, forward=0.0, right=0.0, down=0.0, turn_deg=0.0):
    return (x + forward*math.cos(yaw)-right*math.sin(yaw),
            y + forward*math.sin(yaw)+right*math.cos(yaw),
            z + down,
            yaw + math.radians(turn_deg))


def main():
    ap=argparse.ArgumentParser(description='Simple relative BlueROV2 waypoint test (run controller:=true, autonomy:=false)')
    ap.add_argument('--forward', type=float, default=0.0, help='metres in initial vehicle heading')
    ap.add_argument('--right', type=float, default=0.0, help='metres to starboard in initial heading')
    ap.add_argument('--down', type=float, default=0.0, help='metres, Stonefish NED positive down')
    ap.add_argument('--turn-deg', type=float, default=0.0, help='relative target yaw in degrees')
    ap.add_argument('--seconds', type=float, default=45.0)
    args=ap.parse_args()
    if not all(map(math.isfinite,(args.forward,args.right,args.down,args.turn_deg,args.seconds))):
        ap.error('All inputs must be finite')
    if not 3.0<=args.seconds<=120.0: ap.error('--seconds must be in [3,120]')
    if max(abs(args.forward),abs(args.right))>5 or abs(args.down)>1 or abs(args.turn_deg)>180:
        ap.error('For safety, horizontal <=5 m, down <=1 m, turn <=180 degrees')
    if not any(abs(v)>1e-6 for v in (args.forward,args.right,args.down,args.turn_deg)):
        ap.error('Request at least one motion')

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from .bluerov_waypoint_math import yaw_from_quat
    rclpy.init()
    node=Node('bluerov_waypoint_demo')
    current=[None]
    node.create_subscription(Odometry,'/bluerov2/odometry',
                             lambda m: current.__setitem__(0,m),10)
    pub=node.create_publisher(PoseStamped,'/bluerov2/setpoint',10)
    try:
        deadline=time.monotonic()+8
        while rclpy.ok() and current[0] is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
        if current[0] is None:
            raise RuntimeError('No /bluerov2/odometry received. Is Stonefish running?')
        pose=current[0].pose.pose
        x,y,z=pose.position.x,pose.position.y,pose.position.z
        yaw=yaw_from_quat(pose.orientation)
        gx,gy,gz,gyaw=relative_goal(x,y,z,yaw,args.forward,args.right,args.down,args.turn_deg)
        node.get_logger().info('Initial NED (%.2f %.2f %.2f), yaw %.1f deg; desired (%.2f %.2f %.2f), yaw %.1f deg' %
                               (x,y,z,math.degrees(yaw),gx,gy,gz,math.degrees(gyaw)))
        end=time.monotonic()+args.seconds
        while rclpy.ok() and time.monotonic()<end:
            msg=PoseStamped()
            msg.header.frame_id='world'
            msg.header.stamp=node.get_clock().now().to_msg()
            msg.pose.position.x=gx;msg.pose.position.y=gy;msg.pose.position.z=gz
            msg.pose.orientation.z=math.sin(gyaw/2);msg.pose.orientation.w=math.cos(gyaw/2)
            pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=0.2)
        node.get_logger().info('Waypoint demo finished. Stop PID or send a new setpoint; stale target causes zero thrusters after 5 s.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__': main()
