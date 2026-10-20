#!/usr/bin/env python3
"""End-to-end Stonefish waypoint PID smoke tests (requires live simulator).

Run only with controller:=true, autonomy:=false, clear photogrammetry scene.
Each test is a real closed-loop odometry measurement; offline unit tests do
NOT replace it.  This node commands PoseStamped goals, never direct thrusters.
"""
import argparse
import json
import math
import time
from pathlib import Path

from .bluerov_waypoint_demo import relative_goal
from .bluerov_waypoint_math import wrap_pi, yaw_from_quat


def estimate_pose(msg):
    p=msg.pose.pose.position
    return (float(p.x),float(p.y),float(p.z),yaw_from_quat(msg.pose.pose.orientation))


def pose_error(actual,goal):
    return {'xy_m':round(math.hypot(actual[0]-goal[0],actual[1]-goal[1]),3),
            'depth_m':round(abs(actual[2]-goal[2]),3),
            'yaw_deg':round(abs(math.degrees(wrap_pi(actual[3]-goal[3]))),2)}


def within(goal_type,e):
    if goal_type=='yaw':return e['yaw_deg']<=12 and e['xy_m']<=1.0 and e['depth_m']<=0.35
    if goal_type=='forward':return e['xy_m']<=0.85 and e['depth_m']<=0.35
    if goal_type=='depth':return e['depth_m']<=0.17 and e['xy_m']<=1.0
    return False


def main():
    ap=argparse.ArgumentParser(description='Live BlueROV2 waypoint PID verification (forward, yaw, depth)')
    ap.add_argument('--timeout',type=float,default=55.0,help='Per-test wall-clock timeout [15,120] seconds')
    ap.add_argument('--output',default=str(Path.home()/'roscon_photogrammetry'/'waypoint_pid_test.json'))
    args=ap.parse_args()
    if not math.isfinite(args.timeout) or not 15<=args.timeout<=120:ap.error('--timeout must be 15–120 s')

    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    rclpy.init()
    node=Node('bluerov_waypoint_test')
    pose=[None]
    pose_time=[None]
    def on_odom(msg):
        pose[0]=estimate_pose(msg)
        pose_time[0]=time.monotonic()
    node.create_subscription(Odometry,'/bluerov2/odometry',on_odom,10)
    pub=node.create_publisher(PoseStamped,'/bluerov2/setpoint',10)
    result={'schema':'roscon-waypoint-pid-smoke-v1','config':'simulation only, clear scene, autonomy disabled',
            'steps':[], 'all_passed':False}
    def spin(delay=0.1):rclpy.spin_once(node,timeout_sec=delay)
    def publish(g):
        msg=PoseStamped()
        msg.header.frame_id='world'
        msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=g[:3]
        msg.pose.orientation.z=math.sin(g[3]/2)
        msg.pose.orientation.w=math.cos(g[3]/2)
        pub.publish(msg)
    try:
        wait=time.monotonic()+12.0
        while rclpy.ok() and pose[0] is None and time.monotonic()<wait:spin()
        if pose[0] is None:raise RuntimeError('No /bluerov2/odometry after 12 seconds')
        # Keep the tests short and simple enough to run before any autonomous mission.
        for name,delta in (('yaw',{'turn_deg':30}),('forward',{'forward':3.0}),('depth',{'down':0.35})):
            start=pose[0]
            goal=relative_goal(*start,**delta)
            t0=time.monotonic();last_pub=0.0;stable_since=None;track=[]
            reached=False
            node.get_logger().info('Starting %s test: goal world NED xyz=(%.2f,%.2f,%.2f) yaw=%.1f deg'%
                                   (name,*goal[:3],math.degrees(goal[3])))
            while rclpy.ok() and time.monotonic()-t0<args.timeout:
                now=time.monotonic()
                if now-last_pub>=0.18:
                    publish(goal);last_pub=now
                spin()
                if pose[0] is None or now-(pose_time[0] or 0)>2.0:
                    continue
                e=pose_error(pose[0],goal)
                track.append((round(now-t0,2),tuple(round(x,3) for x in pose[0])))
                if within(name,e):
                    if stable_since is None:stable_since=now
                    if now-stable_since>=1.0:
                        reached=True;break
                else:
                    stable_since=None
            end=pose[0]
            measured={'name':name,'passed':reached,'elapsed_s':round(time.monotonic()-t0,2),
                      'start_xyzyaw_ned':list(start),'requested_xyzyaw_ned':list(goal),
                      'final_xyzyaw_ned':list(end),'final_errors':pose_error(end,goal),
                      'path_samples_n':len(track),
                      'net_xy_m':round(math.hypot(end[0]-start[0],end[1]-start[1]),3),
                      'net_depth_m':round(end[2]-start[2],3),
                      'net_yaw_deg':round(math.degrees(wrap_pi(end[3]-start[3])),2)}
            result['steps'].append(measured)
            node.get_logger().info('%s %s: %s'%(name,'PASS' if reached else 'FAIL',json.dumps(measured['final_errors'])))
            if not reached:break
        result['all_passed']=len(result['steps'])==3 and all(x['passed'] for x in result['steps'])
    except Exception as exc:
        result['error']=str(exc)
        node.get_logger().error(str(exc))
    finally:
        output=Path(args.output).expanduser();output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(result,indent=2)+'\n')
        node.get_logger().info('Test report: %s'%output)
        node.destroy_node()
        rclpy.shutdown()
    if not result['all_passed']:
        raise SystemExit(2)

if __name__=='__main__':main()
