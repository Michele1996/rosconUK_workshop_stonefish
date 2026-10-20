"""Fast waypoint profile, live-test API and unsolved plan submission integration."""
import json
import math
import tempfile
import unittest
from pathlib import Path

from roscon_stonefish_workshop.bluerov_waypoint_math import (
    WaypointGains, allocate, waypoint_efforts)
from roscon_stonefish_workshop.bluerov_waypoint_test import pose_error, within
from roscon_stonefish_workshop.autonomy_geometry import SITES
from roscon_stonefish_workshop.autonomy_plan import load_plan
from roscon_stonefish_workshop.inspection_reference import build_plan

ROOT=Path(__file__).resolve().parents[1]


class FastPidAndCustomPlans(unittest.TestCase):
    def test_faster_cruise_with_braking(self):
        g=WaypointGains()
        self.assertGreaterEqual(g.max_forward,.5)
        self.assertGreaterEqual(g.kp_forward,.25)
        cmd,mode,*_=waypoint_efforts(0,0,6,0,10,0,6,0,gains=g)
        self.assertEqual(mode,'MOVE')
        self.assertGreaterEqual(cmd[0],.5)
        far=cmd[0]
        near=waypoint_efforts(0,0,6,0,1.0,0,6,0,gains=g)[0][0]
        self.assertLess(near,far)
        self.assertEqual(allocate(10,0,0,0,limit=.65)[:4],[.65,.65,-.65,-.65])

    def test_waypoint_smoke_metrics(self):
        e=pose_error((0,.1,6.2,math.radians(32)),(0,0,6.1,math.radians(30)))
        self.assertTrue(within('yaw',e))
        self.assertFalse(within('depth',pose_error((0,0,7,0),(0,0,6,0))))

    def test_empty_templates_are_unsolved(self):
        for target in ('bust','wreck'):
            p=ROOT/f'data/autonomy/{target}_plan_TEMPLATE.json'
            self.assertTrue(p.exists())
            obj=json.loads(p.read_text())
            self.assertEqual(obj['waypoints'],[])
            with self.assertRaisesRegex(ValueError,'no waypoints'):
                load_plan(p,target)

    def test_custom_bust_plan_can_be_checked_and_executed(self):
        # The test synthesises waypoints dynamically; NO bust solution is
        # distributed as a prefilled participant plan.
        cx,cy=SITES['bust']
        pts=[(0,3),(90,3),(180,3),(270,3)]
        q=[{'x':cx-5,'y':cy,'z':6.1,'yaw':0.0,'phase':'SEARCH'},
           {'x':cx+3,'y':cy,'z':7.1,'yaw':math.pi,'phase':'APPROACH'}]
        for az,r in pts:
            t=math.radians(az)
            q.append({'x':cx+r*math.cos(t),'y':cy+r*math.sin(t),
                      'z':7.1,'yaw':t+math.pi,'phase':'INSPECT'})
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'bust.json'
            path.write_text(json.dumps({'target':'bust','frame':'world_ned','waypoints':q}))
            check=load_plan(path,'bust')
            self.assertEqual(check['inspection_views'],4)
            result=build_plan((-16.,-6.,6.1,math.pi),'bust',mode='custom',plan_file=path)
        self.assertEqual(result[-1]['phase'],'RETURN')
        self.assertGreaterEqual(sum(p['phase']=='INSPECT' for p in result),4)
        self.assertTrue(any(p['phase']=='APPROACH' for p in result))

    def test_custom_wreck_plan_supports_distinct_views_and_return(self):
        cx,cy=SITES['wreck']
        # Test-time synthetic geometry, not an authored wreck route shipped
        # in the participant template.
        angles=(0,30,60,120,150,180,240,330)
        q=[{'x':cx-10,'y':cy,'z':6.5,'yaw':0.0,'phase':'SEARCH'},
           {'x':cx+10,'y':cy,'z':7.1,'yaw':math.pi,'phase':'APPROACH'}]
        for deg in angles:
            theta=math.radians(deg)
            q.append({'x':cx+10*math.cos(theta),'y':cy+10*math.sin(theta),
                      'z':7.1,'yaw':theta+math.pi,'phase':'INSPECT'})
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'wreck.json'
            path.write_text(json.dumps({'target':'wreck','frame':'world_ned','waypoints':q}))
            validated=load_plan(path,'wreck')
            self.assertEqual(validated['inspection_views'],8)
            path_goals=build_plan((-16.,-6.,6.1,math.pi),'wreck',mode='custom',plan_file=path)
        self.assertGreaterEqual(sum(g['phase']=='INSPECT' for g in path_goals),8)
        self.assertEqual(path_goals[-1]['phase'],'RETURN')

    def test_invalid_bust_and_wreck_plan_fail_cleanly(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'bad.json'
            path.write_text(json.dumps({'target':'wreck','frame':'world_ned',
                'waypoints':[{'x':13.,'y':2.,'z':7.0,'yaw':0.0,'phase':'INSPECT'}]}))
            with self.assertRaises(ValueError):load_plan(path,'wreck')
            with self.assertRaises(ValueError):build_plan((-16,-6,6.1,0),'bust','custom',plan_file=path)

    def test_installable_entrypoints_and_launch(self):
        setup=(ROOT/'setup.py').read_text()
        self.assertIn('bluerov_waypoint_test =',setup)
        self.assertIn('autonomy_plan_check =',setup)
        launch=(ROOT/'launch/bluerov2_workshop.launch.py').read_text()
        self.assertIn("LaunchConfiguration('plan_file')",launch)
        self.assertNotIn("executable='thruster_adapter'",launch)
        readme=(ROOT/'README.md').read_text()
        self.assertIn('autonomy_plan_check',readme)

if __name__=='__main__':unittest.main()
