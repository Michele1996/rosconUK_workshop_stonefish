#!/usr/bin/env python3
"""Offline guided inspection mission review, separate from photo sampling score."""
import argparse
import json
from pathlib import Path
import math
from .autonomy_geometry import point_clear, segment_clear, SITES, visible


def obstacle_clearance_proxy(trajectory):
    """Estimate unsafe trajectory segments against published static map.

    The result is NOT Stonefish physics contact sensing; sample gaps may miss
    brief collisions. The 0.35 m margin is a conservative vehicle envelope.
    """
    points=[]
    for sample in trajectory:
        if isinstance(sample,(list,tuple)) and len(sample)>=4:
            try: points.append((float(sample[-3]),float(sample[-2]),float(sample[-1])))
            except (ValueError,TypeError): continue
    if not points:return None
    violations=sum(not point_clear(x,y,z,margin=.35) for x,y,z in points)
    for p,q in zip(points,points[1:]):
        if not segment_clear(p,q,z=min(p[2],q[2]),margin=.35):violations+=1
    return {'samples':len(points),'segments':max(0,len(points)-1),
            'unsafe_samples_or_segments':violations,
            'clearance_proxy_pass':violations==0,
            'method':'sampled odometry versus static scene map, NOT actual contact measurement'}


def image_view_proxy(report):
    """Approximate visible novelty of reached inspection poses, in XY only."""
    target=report.get('target');xy=SITES.get(target)
    if xy is None:return None
    visible_sectors=set();reached=0
    for e in report.get('events',[]):
        if e.get('phase')!='INSPECT':continue
        p=e.get('position_ned_m')
        if not isinstance(p,(list,tuple)) or len(p)<3:continue
        x,y,z=map(float,p[:3]);reached+=1
        if not visible((x,y),xy,z):continue
        if target=='wreck': sector=('left' if x<xy[0] else 'right',int(round(y/3)),round(z))
        else:sector=(int((math.atan2(y-xy[1],x-xy[0])+math.pi)*12/(2*math.pi))%12,round(z))
        visible_sectors.add(sector)
    return {'inspection_pose_events':reached,'approximately_visible_unique_sectors':len(visible_sectors),
            'method':'2D occlusion and azimuth/height proxy; NOT true 3D observed surface completeness'}


def evaluate(report, acquisition=None):
    r=json.loads(Path(report).read_text()) if isinstance(report,(str,Path)) else report
    if r.get('schema')!='roscon-inspection-v1':raise ValueError('not an inspection mission report')
    total=max(1,int(r['waypoints_total']));ach=int(r['waypoints_reached'])
    inspections=max(1,int(r['inspection_waypoints_total']))
    covered=int(r['inspection_waypoints_reached'])
    returned=bool(r.get('returned_home'))
    approach=15 if any(e.get('phase')=='APPROACH' for e in r.get('events',[])) else 0
    route=35*min(1,covered/inspections)
    home=25 if returned else 0
    duration=float(r.get('mission_elapsed_wall_s',0))
    efficient=10 if returned and duration<=450 else (5 if returned else 0)
    acquisition_score=None;photos=0
    if acquisition is not None:
        a=json.loads(Path(acquisition).read_text()) if isinstance(acquisition,(str,Path)) else acquisition
        t=r['target']
        if t=='wreck':acquisition_score=a.get('shipwreck',{}).get('score_out_of_100')
        else:acquisition_score=a.get('statues',{}).get({'cat':'cat','bust':'bust'}[t],{}).get('score_out_of_100')
        if acquisition_score is not None:photos=15*max(0,min(100,acquisition_score))/100
    clearance=obstacle_clearance_proxy(r.get('trajectory_samples_ned_m',[]))
    novelty=image_view_proxy(r)
    proxy_penalty=20 if clearance is not None and not clearance['clearance_proxy_pass'] else 0
    return {'schema':'roscon-inspection-score-v1','target':r['target'],
        'mission_status':r.get('status'),
        'score_out_of_100':round(max(0,approach+route+home+efficient+photos-proxy_penalty),2),
        'approach_points_of_15':approach,'inspection_waypoints_points_of_35':round(route,2),
        'return_points_of_25':home,'mission_efficiency_points_of_10':efficient,
        'image_acquisition_points_of_15':round(photos,2),
        'image_score_available':acquisition_score is not None,
        'obstacle_clearance_proxy':clearance,
        'obstacle_clearance_proxy_penalty':proxy_penalty,
        'inspection_viewpoint_novelty_proxy':novelty,
        'limitations':'Known-map score using odometry and sampled visibility proxies; true physical collisions, visual discovery and surface completeness require separate sensors and reconstruction evidence.'}


def main():
    p=argparse.ArgumentParser(description='Evaluate autonomous guided inspection mission')
    p.add_argument('--mission',required=True)
    p.add_argument('--acquisition',help='Optional photogrammetry acquisition_scores.json (15 points)')
    p.add_argument('--output',default='')
    a=p.parse_args()
    result=evaluate(a.mission,a.acquisition)
    txt=json.dumps(result,indent=2)
    print(txt)
    if a.output:
        Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(txt+'\n')

if __name__=='__main__':main()
