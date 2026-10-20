"""Participant-authored map-assisted inspection plans for bust and wreck.

Does not supply a solved bust/wreck plan.  Provides schema validation,
collision/occlusion *proxies* from the published static map, and a reusable
execution contract.  Sensor-driven solutions can create/update the JSON plan
at runtime, or implement an independent PoseStamped publisher.
"""
import argparse
import json
import math
from pathlib import Path

from .autonomy_geometry import OBSTACLES, SITES, point_clear, segment_clear, visible

PHASES = {'SEARCH', 'TRANSIT', 'APPROACH', 'INSPECT'}


def load_plan(path, target=None, obstacles=OBSTACLES):
    raw = json.loads(Path(path).expanduser().read_text())
    if not isinstance(raw, dict) or raw.get('frame') != 'world_ned':
        raise ValueError('Plan must be an object with frame: world_ned')
    name = raw.get('target')
    if name not in ('bust', 'wreck') or (target is not None and name != target):
        raise ValueError('Plan target must match selected bust or wreck')
    values = raw.get('waypoints')
    if not isinstance(values, list) or not values:
        raise ValueError('Plan has no waypoints. Fill the template with search, approach and inspection goals.')
    points = []
    for i, pt in enumerate(values):
        if not isinstance(pt, dict) or pt.get('phase') not in PHASES:
            raise ValueError(f'Waypoint {i}: invalid phase. Choose SEARCH, TRANSIT, APPROACH or INSPECT')
        try:
            q = {k:float(pt[k]) for k in ('x','y','z','yaw')}
        except (KeyError, ValueError, TypeError) as e:
            raise ValueError(f'Waypoint {i}: expected x,y,z,yaw as numbers') from e
        if not all(math.isfinite(x) for x in q.values()):
            raise ValueError(f'Waypoint {i}: nonfinite number')
        if not 3.0 < q['z'] < 12.0:
            raise ValueError(f'Waypoint {i}: depth must be in (3,12) NED metres')
        if not point_clear(q['x'],q['y'],q['z'],margin=0.65,obstacles=obstacles):
            raise ValueError(f'Waypoint {i}: inside a known obstruction, target volume or scene boundary')
        q['phase'] = pt['phase']
        points.append(q)
    phases=[q['phase'] for q in points]
    if 'APPROACH' not in phases:
        raise ValueError('Include at least one APPROACH waypoint before inspection')
    views=[q for q in points if q['phase']=='INSPECT']
    min_views = 4 if name=='bust' else 6
    if len(views)<min_views:
        raise ValueError(f'{name}: provide at least {min_views} INSPECT viewpoints')
    if phases.index('APPROACH') > phases.index('INSPECT'):
        raise ValueError('APPROACH must precede INSPECT')
    for i,p in enumerate(views):
        if not visible((p['x'],p['y']),SITES[name],p['z'],obstacles=obstacles):
            raise ValueError(f'INSPECT waypoint {i} is occluded by the known obstacle proxy')
    # Reject inspection viewpoints all in the same place; otherwise fake
    # novelty from repeated requests would be rewarded.
    sectors=set()
    cx,cy=SITES[name]
    for p in views:
        sectors.add(int(((math.atan2(p['y']-cy,p['x']-cx)+math.pi)%(2*math.pi))/(math.pi/3)))
    if len(sectors)<3:
        raise ValueError('Choose INSPECT viewpoints in at least 3 different azimuth sectors')
    return {'target':name,'frame':'world_ned','waypoints':points,
            'inspection_views':len(views),'inspection_sectors':len(sectors),
            'limitations':'Static known-map/2D line-of-sight proxy; not sensor-based discovery or actual image overlap.'}


def validate_plan(path, target=None):
    return load_plan(path, target)


def main():
    p=argparse.ArgumentParser(description='Check a participant-authored bust/wreck inspection plan before Stonefish execution')
    p.add_argument('--plan',required=True,help='JSON plan path (see data/autonomy templates)')
    p.add_argument('--target',choices=['bust','wreck'])
    args=p.parse_args()
    try:
        q=validate_plan(args.plan,args.target)
    except (ValueError,OSError,json.JSONDecodeError) as e:
        print('PLAN INVALID:',e)
        raise SystemExit(2)
    print(json.dumps({'status':'PLAN_VALID','target':q['target'],
        'waypoints':len(q['waypoints']),'inspection_views':q['inspection_views'],
        'inspection_azimuth_sectors':q['inspection_sectors'],
        'note':'A geometric check is not proof the BlueROV can follow the route or capture usable photographs.'},indent=2))

if __name__=='__main__':main()
