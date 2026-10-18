#!/usr/bin/env python3
"""Create the ROSCon BlueBoat snail course with tangent-continuous bends.

Keeps the existing two S bends; puts 20m of straight track after START,
then a 258 degree inward snail, a 10m straight, a 90 degree elbow and
25m finishing straight.  The route is dense sampled so the judge, PID and
buoy generator all use the exact same geometry (no hidden interpolation).
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config' / 'blueboat_race.json'


def bezier(a, b, c, d, n=51):
    out = []
    for i in range(1, n):
        t = i / (n-1)
        v = (1-t)**3, 3*t*(1-t)**2, 3*t*t*(1-t), t**3
        out.append([sum(w*p[j] for w,p in zip(v,(a,b,c,d))) for j in (0,1)])
    return out


def route():
    pts = [[-4.0,0.0]]
    sections = {}
    def add(name, q):
        start = len(pts)-1
        for p in q:
            pp = [float(p[0]),float(p[1])]
            if math.dist(pp,pts[-1])>1e-7:
                pts.append(pp)
        sections[name] = (start,len(pts)-1)

    # START at s=6: 20m of straight remains until x=22.
    add('initial_20m_straight', [[-4+26*i/40,0] for i in range(1,41)])
    # The same two alternating S-shaped changes of heading as the old circuit.
    add('original_S_bend_1',bezier([22,0],[29,0],[29,8],[36,8],55))
    add('original_middle_straight',[[36+8*i/16,8] for i in range(1,17)])
    add('original_S_bend_2',bezier([44,8],[51,8],[55,-6],[64,-6],65))
    add('original_approach',[[64+18*i/36,-6] for i in range(1,37)])

    # Clockwise, inward Archimedean spiral: radius 24 -> 12.75m,
    # 4.5 rad = ~258 degrees.  Crucially OPEN (not a closed circle),
    # leaving the final leg free without crossing the initial approach.
    r0, k, total_angle = 24.0, -2.5, 4.5
    origin = (82., -6.)
    # Initial outward X tangent is matched exactly to the old straight.
    initial_dyaw = math.atan2(-k,r0)
    ca, sa = math.cos(-initial_dyaw), math.sin(-initial_dyaw)
    spiral=[]
    for i in range(1,361):
        theta=total_angle*i/360
        r=r0+k*theta
        x=r*math.sin(theta)
        y=r0-r*math.cos(theta)
        rx=ca*x-sa*y
        ry=sa*x+ca*y
        # Reflect across local X for clockwise motion.
        spiral.append([origin[0]+rx,origin[1]-ry])
    add('inward_spiral_258deg',spiral)

    # Precisely 10m after the spiral, tangent to its exit.
    t0=math.atan2(pts[-1][1]-pts[-2][1],pts[-1][0]-pts[-2][0])
    start=pts[-1].copy()
    heading=(math.cos(t0),math.sin(t0))
    add('post_spiral_10m_straight',[[start[j]+heading[j]*(10*i/20) for j in (0,1)] for i in range(1,21)])

    # 90-degree LEFT elbow, radius 8.5m: not a lane-crossing hairpin.
    start=pts[-1].copy()
    radius=8.5
    add('elbow_90deg',[
        [start[0]+radius*(math.sin(t0+d)-math.sin(t0)),
         start[1]+radius*(-math.cos(t0+d)+math.cos(t0))]
        for d in (math.pi/2*i/60 for i in range(1,61))])
    start=pts[-1].copy()
    heading=(math.cos(t0+math.pi/2),math.sin(t0+math.pi/2))
    add('finish_25m_straight',[[start[j]+heading[j]*(25*i/50) for j in (0,1)] for i in range(1,51)])

    distances=[0.0]
    for p,q in zip(pts,pts[1:]):
        distances.append(distances[-1]+math.dist(p,q))
    stage={k:{'start_s_m':round(distances[a],3),'end_s_m':round(distances[b],3),
              'length_m':round(distances[b]-distances[a],3)} for k,(a,b) in sections.items()}
    return pts,stage,distances[-1]


def main():
    c=json.loads(CONFIG.read_text())
    pts, stage, total=route()
    c.update({
        'centerline_xy_m': [[round(x,6),round(y,6)] for x,y in pts],
        'centerline_interpolation': 'sampled',
        'course_version': 'v4_20m_S_bends_inward_snail_10m_elbow_25m',
        'half_width_m': 3.4,
        'boat_boundary_margin_m': 0.4,
        'buoy_spacing_m': 1.8,
        'boat_start_xyz_m': [-3.0,0.0,0.1],
        'start_gate_s_m': 6.0,
        'checkpoint_s_m': [18,35,52,70,89,105,123,141,158,175,191,211],
        'finish_gate_s_m': round(total-2.5,3),
        'max_race_time_s': 1200.0,
        'spiral_angle_deg': round(4.5*180/math.pi,2),
        'spiral_radius_start_end_m': [24.0,12.75],
        'elbow_radius_m': 8.5,
        'course_sections': stage,
        'roadside_aruco_markers': '12 paired checkpoint gates: right IDs 11-22; left IDs 31-42. 1.60m original panels, centre-fitted codes, separate arrow headers.',
        'description': '20m straight after START -> existing S-bends -> clockwise inward ~258-degree snail -> 10m straight -> 90-degree elbow -> 25m finish straight.',
    })
    c.pop('elbow_approx_radius_m',None)
    c.pop('elbow_description',None)
    CONFIG.write_text(json.dumps(c,indent=2)+'\n')
    print('Wrote v4 geometry:', round(total,2),'metres; stages:',stage)

if __name__ == '__main__':
    main()
