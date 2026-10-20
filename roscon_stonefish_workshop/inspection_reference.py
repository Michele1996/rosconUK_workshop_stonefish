#!/usr/bin/env python3
"""Challenge 3 autonomy: complete MAP-ASSISTED CAT reference only.

CAT reference: fully automatic waypoint execution using odometry and a PRIOR
map of static obstacles plus the cat nominal position. A* obstacle-aware routes
and approximate occlusion-aware viewpoint selection are implemented.
BUST/WRECK starter: short search only; autonomous inspection, safe transit,
and return are participant implementation tasks, NOT solved here.

This does NOT solve unconstrained object recognition, unknown-scene mapping,
true mesh-surface coverage, or dynamic obstacle avoidance. Visual ArUco
recognition may optionally be made mandatory with visual_required:=true.
"""
import json
import math
import time
from pathlib import Path
from .autonomy_geometry import (SITES,OBSTACLES,astar,point_clear,select_views,
                                  point_segment_distance,visible)
from .autonomy_exercises import choose_next_best_views
from .autonomy_plan import load_plan


def goal(x,y,z,yaw,phase):
    return {'x':float(x),'y':float(y),'z':float(z),'yaw':float(yaw),'phase':phase}


def angle_to(x,y,target):
    cx,cy=SITES[target];return math.atan2(cy-y,cx-x)


def build_plan(home,target,mode='reference',obstacles=OBSTACLES,search=True,plan_file=None):
    """Return checked navigation goals, including explicit RETURN at end.

    Routes are rerouted at each new goal using A*. The planner uses a fixed
    obstacle prior; do NOT claim this is sensor-based unknown-obstacle avoidance.
    """
    if target not in SITES:raise ValueError('Unknown target '+str(target))
    if mode not in ('reference','starter','custom'):raise ValueError('Unknown planner mode')
    if mode == 'reference' and target != 'cat':
        raise ValueError('Fully solved reference is intentionally CAT-ONLY. Use mode=starter for bust/wreck.')
    cx,cy=SITES[target]
    hx,hy,hz,hyaw=home
    z_search=6.9 if target!='wreck' else 6.6
    destinations=[]
    custom_yaws=[]
    if mode=='custom':
        if not plan_file: raise ValueError('Custom mode requires plan_file with participant-authored JSON')
        participant=load_plan(plan_file,target,obstacles)
        for p in participant['waypoints']:
            destinations.append((p['x'],p['y'],p['z'],p['phase']))
            custom_yaws.append(p['yaw'])
        # Return is added by generic runner; the participant must provide
        # search, approach and safe/visible INSPECT viewpoint choices.
        destinations.append((hx,hy,hz,'RETURN'))
        custom_yaws.append(hyaw)
        search=False
    if search:
        for i,(dx,dy) in enumerate([(-5,-4),(-5,4),(4,-4),(4,4)]):
            x,y=cx+dx,cy+dy
            # Hold launch depth at the first search waypoint: avoid a simultaneous
            # initial 100-degree turn, translation AND dive. Subsequent search
            # viewpoints use the planned survey depth.
            z=hz if i==0 else z_search
            if point_clear(x,y,z,obstacles=obstacles):
                destinations.append((x,y,z,'SEARCH'))
    if mode == 'reference':
        views=select_views('cat',18,obstacles)
        if not views:raise RuntimeError('No safe visible cat viewpoints')
        destinations.append((*views[0],'APPROACH'))
        for x,y,z in views:
            destinations.append((x,y,z,'INSPECT'))
        destinations.append((hx,hy,hz,'RETURN'))
    elif mode=='starter':
        # SEARCH-ONLY starter: no approach, view planner, or return solution.
        # Implement those in autonomy_exercises.py and then integrate here.
        if not destinations:
            raise RuntimeError('No safe search positions available')
    output=[];current=(hx,hy,hz)
    for destination_index,(x,y,z,phase) in enumerate(destinations):
        # A* with target-independent NED vertical coordinate, planning around
        # obstacles from surface to seabed; changes in z happen after routing.
        route=astar(current[:2],(x,y),z,obstacles=obstacles)
        for i,(px,py) in enumerate(route[1:]):
            is_last=i==len(route)-2
            phase_label=phase if is_last else 'TRANSIT'
            heading = (custom_yaws[destination_index] if mode=='custom' and is_last
                       else hyaw if phase_label=='RETURN' else angle_to(px,py,target))
            output.append(goal(px,py,z,heading,phase_label))
        current=(x,y,z)
    return output


def reached(position,goal,h_tol=0.75,z_tol=0.50):
    return (math.hypot(position[0]-goal['x'],position[1]-goal['y'])<h_tol and
            abs(position[2]-goal['z'])<z_tol)


def reached_inspection_pose(position,orientation,pt,angle_tolerance=0.38):
    """Require camera heading at inspection poses, not only position."""
    return reached(position,pt) and abs(math.atan2(
        math.sin(orientation_yaw(orientation)-pt['yaw']),
        math.cos(orientation_yaw(orientation)-pt['yaw'])))<angle_tolerance


def orientation_yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class ReferenceMission(Node):
        def __init__(self):
            super().__init__('bluerov_inspection_reference')
            for name,val in {
                'target':'cat','mode':'reference','visual_required':False,
                'odom_topic':'/bluerov2/odometry', 'setpoint_topic':'/bluerov2/setpoint',
                'detections_topic':'/bluerov2/autonomy/detections',
                'max_mission_seconds':1350.0,'waypoint_timeout_seconds':150.0,
                'plan_file':'',
                'output_dir':str(Path.home()/'roscon_photogrammetry'/'missions'),
            }.items():self.declare_parameter(name,val)
            self.target=str(self.get_parameter('target').value)
            self.mode=str(self.get_parameter('mode').value)
            if self.target not in SITES:raise ValueError('Target must be cat, bust or wreck')
            if self.mode == 'reference' and self.target != 'cat':
                raise ValueError('The full reference solves CAT ONLY. For bust/wreck set autonomy_mode:=starter')
            if self.mode not in ('reference','starter','custom'):
                raise ValueError('autonomy_mode must be reference, starter or custom')
            self.visual_required=bool(self.get_parameter('visual_required').value)
            self.plan_file=str(self.get_parameter('plan_file').value)
            self.pub=self.create_publisher(PoseStamped,self.get_parameter('setpoint_topic').value,10)
            self.status=self.create_publisher(String,'/bluerov2/inspection/status',10)
            self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.on_odom,10)
            self.create_subscription(String,self.get_parameter('detections_topic').value,self.on_detection,10)
            self.create_timer(0.5,self.tick)
            self.position=None;self.q=None;self.home=None;self.path=[];self.index=0
            self.started=None;self.segment_since=None;self.done=False
            self.events=[];self.trajectory=[];self.last_sample=0.0
            self.seen=False;self.observations=[];self.search_completed=False
            self.output_dir=Path(self.get_parameter('output_dir').value).expanduser()
            self.output_dir.mkdir(parents=True,exist_ok=True)
            self.get_logger().info('Autonomy %s target %s: known nominal SITE + KNOWN STATIC OBSTACLE MAP; visual_required=%s'%(
                self.mode,self.target,self.visual_required))

        def on_odom(self,msg):
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            self.position=(p.x,p.y,p.z);self.q=(q.x,q.y,q.z,q.w)
            now=time.monotonic()
            if now-self.last_sample>=.5:
                self.trajectory.append([round(now-self.started,2) if self.started else 0.0,*map(float,self.position)])
                self.last_sample=now
        def on_detection(self,msg):
            try:
                d=json.loads(msg.data)
                if not isinstance(d,dict):return
                self.observations.append(d)
                self.observations=self.observations[-20:]
                # Detection contract: a participant's actual vision node may
                # publish {"target":"bust", "detected":true,"confidence":0.8}.
                # Never infer detection from nominal site positions.
                if (d.get('target')==self.target and d.get('detected') is True
                        and float(d.get('confidence',0))>=0.65):
                    self.seen=True
            except (ValueError,TypeError):return
        def publish_goal(self,pt):
            msg=PoseStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='world'
            msg.pose.position.x=pt['x'];msg.pose.position.y=pt['y'];msg.pose.position.z=pt['z']
            msg.pose.orientation.z=math.sin(pt['yaw']/2);msg.pose.orientation.w=math.cos(pt['yaw']/2)
            self.pub.publish(msg)
        def emit_status(self,phase,**extra):
            msg=String();msg.data=json.dumps({'state':phase,'target':self.target,'goal_index':self.index,
                'goal_count':len(self.path),'visual_detection':self.seen,**extra})
            self.status.publish(msg)
        def end(self,status,reason):
            if self.done:return
            self.done=True
            report={'schema':'roscon-inspection-v1','target':self.target,
                    'autonomy_mode':'map_assisted_'+self.mode,
                    'status':status,'reason':reason,
                    'mission_elapsed_wall_s':round(time.monotonic()-self.started,2) if self.started else 0,
                    'waypoints_reached':len(self.events),'waypoints_total':len(self.path),
                    'inspection_waypoints_total':sum(g['phase']=='INSPECT' for g in self.path),
                    'inspection_waypoints_reached':sum(e['phase']=='INSPECT' for e in self.events),
                    'returned_home':status=='FINISHED' and self.mode in ('reference','custom'), 'visual_detection':self.seen,
                    'visual_required':self.visual_required,
                    'events':self.events,'trajectory_samples_ned_m':self.trajectory,
                    'known_map_obstacle_count':len(OBSTACLES),
                    'limitations':'CAT uses known nominal site and static obstacles. CUSTOM executes a user-supplied plan with known-map A* transit; it does NOT establish actual visual discovery or true 3D visibility. Bust/wreck starter provides search only. No simulated physical-contact verification is claimed.'}
            path=self.output_dir/('inspection_%s_%s.json'%(self.target,time.strftime('%Y%m%d_%H%M%S')))
            path.write_text(json.dumps(report,indent=2)+'\n')
            self.emit_status(status,reason=reason,report=str(path))
            self.get_logger().info('%s: %s; %s'%(status,reason,path))
            if self.position is not None:self.publish_goal(goal(*self.position,orientation_yaw(self.q),'HOLD'))
        def tick(self):
            if self.done or self.position is None:return
            now=time.monotonic()
            if self.started is None:
                self.started=self.segment_since=now
                self.home=(*self.position,orientation_yaw(self.q))
                try:self.path=build_plan(self.home,self.target,self.mode,plan_file=self.plan_file)
                except (ValueError,RuntimeError) as exc:
                    self.end('PLAN_FAILED',str(exc));return
                self.get_logger().info('%s mission starts with %d waypoints'%(self.target,len(self.path)))
            if now-self.started>float(self.get_parameter('max_mission_seconds').value):
                self.end('TIMEOUT','mission time limit');return
            if self.index>=len(self.path):
                if self.mode == 'starter':
                    self.end('STARTER_READY','search completed; implement recognition, safe approach, viewpoints and return');return
                self.end('FINISHED','completed inspected route and returned home');return
            pt=self.path[self.index]
            # Optional detection gate: no detector provided here; integrate your own.
            # Never transition from SEARCH to
            # APPROACH unless the requested fiducial has been observed.
            if self.visual_required and pt['phase'] not in ('SEARCH','TRANSIT') and not self.seen:
                self.end('TARGET_NOT_DETECTED','No valid camera detection during search');return
            self.publish_goal(pt)
            if (reached_inspection_pose(self.position,self.q,pt) if pt['phase']=='INSPECT'
                else reached(self.position,pt)):
                self.events.append({'phase':pt['phase'],'index':self.index,
                        'elapsed_wall_s':round(now-self.started,2),
                        'position_ned_m':list(map(float,self.position))})
                self.index+=1;self.segment_since=now
                if self.index==len(self.path):
                    if self.mode == 'starter':
                        self.end('STARTER_READY','search completed; finish autonomy implementation');return
                    self.end('FINISHED','completed inspected route and returned home');return
            elif now-self.segment_since>float(self.get_parameter('waypoint_timeout_seconds').value):
                self.end('TIMEOUT','Waypoint %d (%s) missed: horizontal %.2f m, depth error %.2f m'%
                         (self.index,pt['phase'],math.hypot(self.position[0]-pt['x'],self.position[1]-pt['y']),
                          self.position[2]-pt['z']));return
            self.emit_status(pt['phase'])

    rclpy.init();node=ReferenceMission()
    try:rclpy.spin(node)
    except KeyboardInterrupt:node.end('ABORTED','interrupted')
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
