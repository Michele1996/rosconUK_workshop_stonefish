#!/usr/bin/env python3
"""Challenge 3: odometry-guided underwater inspection and return baseline.

This is an executable waypoint/state-machine baseline. It knows the target's
nominal survey SITE; it does NOT claim camera recognition or map-free discovery.
Requires bluerov_pid + thruster_adapter (launch controller:=true) and a separate
photogrammetry_capture node (capture:=true). No simulator truth topic is used.
"""
import json
import math
import time
from pathlib import Path

TARGETS={'cat':(-10.,-6.), 'bust':(-10.,5.), 'wreck':(19.,6.)}


def yaw_towards(x,y,cx,cy):
    return math.atan2(cy-y,cx-x)


def waypoint(x,y,z,yaw,phase):
    return {'x':float(x),'y':float(y),'z':float(z),'yaw':float(yaw),'phase':phase}


def inspection_path(target,home):
    if target != 'cat':
        raise ValueError('Old guided mission is CAT-only. For bust/wreck use autonomy_mode:=starter and implement autonomy_exercises.py')
    cx,cy=TARGETS[target]
    path=[]
    if target!='wreck':
        # First search the neighbourhood rather than immediately start a blind orbit.
        for dx,dy in [(-4,-3),(-4,0),(-4,3)]:
            x,y=cx+dx,cy+dy
            path.append(waypoint(x,y,6.9,yaw_towards(x,y,cx,cy),'SEARCH'))
        path.append(waypoint(cx-3,cy,7.1,0.,'APPROACH'))
        # NED z depth, use two observation heights.
        for z in (7.1,8.0):
            for i in range(12):
                a=math.pi+2*math.pi*i/12
                x,y=cx+3.0*math.cos(a),cy+3.0*math.sin(a)
                path.append(waypoint(x,y,z,yaw_towards(x,y,cx,cy),'INSPECT'))
    hx,hy,hz,hyaw=home
    # Return via a safe stand-off (site route still requires collision testing).
    path.append(waypoint(hx,hy,hz,hyaw,'RETURN'))
    return path


def reached(p,goal,horizontal_tol=0.7,vertical_tol=0.45):
    return math.hypot(p[0]-goal['x'],p[1]-goal['y'])<horizontal_tol and abs(p[2]-goal['z'])<vertical_tol


def quat_yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.node import Node

    class Mission(Node):
        def __init__(self):
            super().__init__('bluerov_inspection_mission')
            self.declare_parameter('target','cat')
            self.declare_parameter('odom_topic','/bluerov2/odometry')
            self.declare_parameter('setpoint_topic','/bluerov2/setpoint')
            self.declare_parameter('max_mission_seconds',750.0)
            self.declare_parameter('waypoint_timeout_seconds',75.0)
            self.declare_parameter('output_dir',str(Path.home()/'roscon_photogrammetry'/'missions'))
            self.target=self.get_parameter('target').value
            if self.target != 'cat':raise ValueError('Legacy guided reference supports cat only; use the obstacle scene starter for bust/wreck')
            self.pub=self.create_publisher(PoseStamped,self.get_parameter('setpoint_topic').value,10)
            self.status_pub=self.create_publisher(String,'/bluerov2/inspection/status',10)
            self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.on_odom,10)
            self.create_timer(.5,self.tick)
            self.position=None;self.orientation=None;self.path=None;self.index=0
            self.start=None;self.segment_start=None;self.arrived=0;self.inspection_goals=0
            self.events=[];self.distance_m=0.;self.last_position=None;self.done=False
            self.record_dir=Path(self.get_parameter('output_dir').value).expanduser()
            self.record_dir.mkdir(parents=True,exist_ok=True)
            self.get_logger().info('Inspection mission target=%s; GUIDED site positions, not visual target detection'%self.target)
        def on_odom(self,msg):
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            pos=(float(p.x),float(p.y),float(p.z))
            if self.last_position is not None and math.dist(pos,self.last_position)<2.0:
                self.distance_m+=math.dist(pos,self.last_position)
            self.last_position=pos
            self.position=pos;self.orientation=(q.x,q.y,q.z,q.w)
        def publish_goal(self,goal):
            p=PoseStamped();p.header.stamp=self.get_clock().now().to_msg();p.header.frame_id='world'
            p.pose.position.x=goal['x'];p.pose.position.y=goal['y'];p.pose.position.z=goal['z']
            yaw=goal['yaw'];p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            self.pub.publish(p)
        def finish(self,status,reason):
            if self.done:return
            self.done=True
            elapsed=time.monotonic()-self.start if self.start else 0.
            result={'schema':'roscon-inspection-v1','target':self.target,
                    'autonomy_mode':'guided_odom_waypoint_baseline',
                    'status':status,'reason':reason,'mission_elapsed_wall_s':round(elapsed,2),
                    'path_distance_odom_m':round(self.distance_m,2),
                    'waypoints_reached':self.arrived,'waypoints_total':len(self.path or []),
                    'inspection_waypoints_total':self.inspection_goals,
                    'inspection_waypoints_reached':sum(1 for e in self.events if e['phase']=='INSPECT'),
                    'returned_home':status=='FINISHED', 'events':self.events,
                    'limitations':'No visual target recognition, surface visibility or obstacle avoidance is claimed.'}
            suffix=time.strftime('%Y%m%d_%H%M%S')
            path=self.record_dir/f'inspection_{self.target}_{suffix}.json'
            path.write_text(json.dumps(result,indent=2)+'\n')
            self.get_logger().info('Inspection %s: %s; report %s'%(status,reason,path))
            msg=String();msg.data=json.dumps({'state':status,'target':self.target,'reason':reason});self.status_pub.publish(msg)
            # Hold last position on completion to avoid stale drive command.
            if self.position is not None:
                self.publish_goal(waypoint(*self.position, quat_yaw(self.orientation),'HOLD'))
        def tick(self):
            if self.done or self.position is None:return
            now=time.monotonic()
            if self.path is None:
                self.start=self.segment_start=now
                self.path=inspection_path(self.target, (*self.position,quat_yaw(self.orientation)))
                self.inspection_goals=sum(g['phase']=='INSPECT' for g in self.path)
                self.get_logger().info('Mission starts: %s with %d goals'%(self.target,len(self.path)))
            if now-self.start>float(self.get_parameter('max_mission_seconds').value):
                self.finish('TIMEOUT','global mission time exceeded');return
            if self.index>=len(self.path):
                self.finish('FINISHED','reached home after inspection');return
            goal=self.path[self.index]
            self.publish_goal(goal)
            if reached(self.position,goal):
                self.events.append({'phase':goal['phase'],'goal_index':self.index,'elapsed_wall_s':round(now-self.start,2),
                                    'position_ned_m':list(self.position)})
                self.arrived+=1;self.index+=1;self.segment_start=now
                if self.index>=len(self.path):self.finish('FINISHED','all waypoints and return completed')
                else:self.get_logger().info('%s goal %d/%d reached'%(goal['phase'],self.index,len(self.path)))
            elif now-self.segment_start>float(self.get_parameter('waypoint_timeout_seconds').value):
                self.finish('TIMEOUT','goal %d %s not reached'%(self.index,goal['phase']))
            state=String();state.data=json.dumps({'state':goal['phase'],'target':self.target,
                        'goal_index':self.index,'goal_count':len(self.path),'position_ned_m':self.position})
            self.status_pub.publish(state)

    rclpy.init();n=Mission()
    try:rclpy.spin(n)
    except KeyboardInterrupt:n.finish('ABORTED','stopped by operator')
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
