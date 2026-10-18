"""Offline tests: python3 -m unittest discover -s tests -v (without ROS)."""
import json
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from roscon_stonefish_workshop.race_engine import RaceEngine
from roscon_stonefish_workshop.race_geometry import (crossing_fraction, gates,
    load_course, length, nearest_centerline, sample_course)


class RaceTests(unittest.TestCase):
    def setUp(self):
        self.course = load_course(PROJECT/'config'/'blueboat_race.json')

    def test_course_and_waypoints(self):
        self.assertGreater(length(self.course), 80)
        self.assertEqual(len(gates(self.course)), 14)
        for i, (name, s, xy, tangent, normal) in enumerate(gates(self.course)):
            self.assertAlmostEqual(math.hypot(*tangent), 1.0, places=6)
            self.assertAlmostEqual(math.hypot(*normal), 1.0, places=6)
            self.assertAlmostEqual(sum(a*b for a,b in zip(tangent,normal)),0,places=6)
            self.assertLess(nearest_centerline(self.course,xy)[0],1e-5)

    def test_practice_ignores_green_start_and_all_checkpoints(self):
        engine = RaceEngine(self.course)
        engine.disarm()
        for i in range(2001):
            p = sample_course(self.course, length(self.course)*i/2000)[0]
            status = engine.update(p, float(i)*0.05)
        self.assertEqual(status['state'], 'practice')
        self.assertEqual(status['gates_passed'], 0)
        self.assertFalse(status['race_started'])
        self.assertEqual(status['adjusted_time_s'], 0)
        engine.arm()
        for i in range(2001):
            p = sample_course(self.course, length(self.course)*i/2000)[0]
            status = engine.update(p, float(i)*0.05 + 200)
        self.assertEqual(status['state'], 'finished')
        self.assertTrue(status['race_started'])

    def test_full_ontrack_finish(self):
        engine = RaceEngine(self.course)
        for i in range(2001):
            p = sample_course(self.course,length(self.course)*i/2000)[0]
            status = engine.update(p, float(i)*0.05)
        self.assertEqual(status['state'], 'finished')
        self.assertEqual(status['gates_passed'],14)
        self.assertEqual(status['offtrack_excursions'],0)
        self.assertAlmostEqual(status['penalty_s'],0)
        self.assertGreater(status['race_time_s'],0)

    def test_missed_first_checkpoint_prevents_finish(self):
        engine = RaceEngine(self.course)
        g = gates(self.course)
        # Crossing start gate followed by teleport to finish does not score a finish.
        engine.update((-2.0,0.0),0.0)
        engine.update((4.0,0.0),1.0)
        self.assertEqual(engine.state,'running')
        for j in range(1,31):
            p = sample_course(self.course, length(self.course)*(j/30))[0]
            engine.update(p,1.0+j*0.1)
        # Since this trajectory also passes sequential checkpoints, it can finish.
        self.assertIn(engine.state, ('finished','running'))
        # Verify that a direct jump from just after START to beyond FINISH does not count.
        r2 = RaceEngine(self.course)
        r2.update((-2.,0.),0.)
        r2.update((4.,0.),1.)
        r2.update((70.,2.),2.)
        self.assertEqual(r2.state,'running')
        self.assertLess(r2.next_index,len(gates(self.course))-1)

    def test_gate_far_extension_cannot_pass(self):
        start = gates(self.course)[0]
        c,t,n = start[2:]
        width = self.course['half_width_m'] - self.course['boat_boundary_margin_m']
        a=(c[0]-t[0]+n[0]*10,c[1]-t[1]+n[1]*10)
        b=(c[0]+t[0]+n[0]*10,c[1]+t[1]+n[1]*10)
        self.assertIsNone(crossing_fraction(a,b,start,width))

    def test_offtrack_penalty(self):
        engine = RaceEngine(self.course)
        engine.update((-2.,0.),0.)
        engine.update((4.,0.),1.)
        engine.update((5.,0.),2.)
        engine.update((6.,5.),3.)
        engine.update((6.,5.),4.)
        engine.update((6.,0.),5.)
        status=engine.status(5.,(6.,0.))
        self.assertEqual(status['offtrack_excursions'],1)
        self.assertGreaterEqual(status['penalty_s'],12.)
        self.assertFalse(status['outside'])

    def test_clock_reset(self):
        engine = RaceEngine(self.course)
        engine.update((-2.,0.),5.)
        engine.update((4.,0.),6.)
        self.assertEqual(engine.state,'running')
        engine.update((-3.,0.),0.)
        self.assertEqual(engine.state,'waiting_for_start')

    def test_extended_elbow_and_both_boundary_rows(self):
        self.assertGreater(length(self.course), 220.)
        self.assertLess(length(self.course), 250.)
        stages=self.course['course_sections']
        self.assertAlmostEqual(stages['initial_20m_straight']['end_s_m'] - self.course['start_gate_s_m'],20,places=2)
        self.assertAlmostEqual(stages['post_spiral_10m_straight']['length_m'],10,places=2)
        self.assertAlmostEqual(stages['finish_25m_straight']['length_m'],25,places=2)
        self.assertAlmostEqual(self.course['spiral_angle_deg'],257.83,places=1)
        self.assertAlmostEqual(stages['elbow_90deg']['length_m'], 8.5*math.pi/2, delta=.03)
        root = ET.parse(PROJECT/'data/scenarios/blueboat_race.scn').getroot()
        objects = root.findall('animated')
        left = sorted([o for o in objects if o.get('name','').startswith('Buoy_L_')], key=lambda x: x.get('name'))
        right = sorted([o for o in objects if o.get('name','').startswith('Buoy_R_')], key=lambda x: x.get('name'))
        self.assertEqual(len(left), len(right))
        self.assertGreater(len(left), 120)
        half = self.course['half_width_m']
        for l, r in zip(left, right):
            pa = [float(v) for v in l.find('trajectory/keypoint').get('xyz').split()]
            pb = [float(v) for v in r.find('trajectory/keypoint').get('xyz').split()]
            self.assertAlmostEqual(math.dist(pa[:2], pb[:2]), 2*half, places=2)
            self.assertEqual(l.get('collisions'), 'false')
            self.assertEqual(r.get('collisions'), 'false')

    def test_spiral_is_noncrossing_and_buoys_both_sides(self):
        # Avoid shortcuts where the official centerline contains a crossing.
        total=length(self.course)
        samples=[(s,sample_course(self.course,s)[0]) for s in
                 [total*i/230 for i in range(231)]]
        for i,(si,a) in enumerate(samples):
            for sj,b in samples[i+1:]:
                if sj-si>19.:
                    self.assertGreater(math.dist(a,b),7.8,
                                       msg=f'Overlapping lanes at {si:.1f} and {sj:.1f}')

    def test_scenario_animation_noncollision_and_spawn(self):
        root=ET.parse(PROJECT/'data/scenarios/blueboat_race.scn').getroot()
        objects=root.findall('animated')
        self.assertGreater(len(objects),70)
        self.assertTrue(all(o.attrib.get('collisions')=='false' for o in objects))
        self.assertTrue(all(o.find('trajectory/keypoint') is not None for o in objects))
        self.assertEqual(len([o for o in objects if o.attrib['name'].startswith('Aruco_')]),24)
        robot=ET.parse(PROJECT/'data/robots/blueboat.scn').getroot().find('robot')
        self.assertEqual(robot.find('world_transform').attrib['xyz'],'-3.0 0.0 0.1')
        publishers = [a.get('topic') for a in robot.findall('sensor/ros_publisher')]
        self.assertIn('/blueboat/race/ground_truth_odom',publishers)
        self.assertIn('/blueboat/odom',publishers)


if __name__=='__main__':
    unittest.main()
