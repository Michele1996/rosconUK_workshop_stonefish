import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from roscon_stonefish_workshop.autonomy_geometry import (
    OBSTACLES, select_views, visible, point_clear, segment_clear, SITES,
)
from roscon_stonefish_workshop.inspection_reference import build_plan
from roscon_stonefish_workshop.inspection_evaluation import obstacle_clearance_proxy, image_view_proxy

ROOT = Path(__file__).resolve().parents[1]

class CatReferenceOnly(unittest.TestCase):
    def test_scene_has_collidable_occluding_barriers_for_all_targets(self):
        root=ET.parse(ROOT/'data/scenarios/bluerov2_autonomy_obstacles.scn').getroot()
        static={x.get('name'):x for x in root.findall('.//static')}
        self.assertGreaterEqual(len(OBSTACLES), 11)
        self.assertTrue(all(o.name in static for o in OBSTACLES))
        self.assertTrue(all(static[o.name].get('type')=='box' for o in OBSTACLES))
        self.assertTrue(all(not static[o.name].get('collisions')=='false' for o in OBSTACLES))
        self.assertIn('CatOccluder',static)
        self.assertIn('BustOccluder',static)
        self.assertIn('WreckWestScreen',static)

    def test_cat_route_is_complete_and_clear(self):
        home=(-16.,-6.,6.1,math.pi)
        path=build_plan(home,'cat',mode='reference')
        phases=[p['phase'] for p in path]
        self.assertIn('SEARCH',phases)
        self.assertIn('APPROACH',phases)
        self.assertGreaterEqual(phases.count('INSPECT'),12)
        self.assertEqual(phases[-1],'RETURN')
        prev=home[:3]
        for p in path:
            cur=(p['x'],p['y'],p['z'])
            self.assertTrue(point_clear(*cur,margin=.55),cur)
            self.assertTrue(segment_clear(prev,cur,z=cur[2],margin=.55),(prev,cur))
            prev=cur
        self.assertLess(math.dist(prev,home[:3]),.01)

    def test_viewpoint_planning_handles_occlusion(self):
        views=select_views('cat',16)
        self.assertGreaterEqual(len(views),12)
        for p in views:
            self.assertTrue(point_clear(*p,margin=.55))
            self.assertTrue(visible(p,SITES['cat'],p[2]))

    def test_other_targets_have_only_search_starter(self):
        home=(-16.,-6.,6.1,math.pi)
        for target in ('bust','wreck'):
            with self.assertRaises(ValueError):build_plan(home,target,'reference')
            stages=[p['phase'] for p in build_plan(home,target,'starter')]
            self.assertIn('SEARCH',stages)
            self.assertNotIn('APPROACH',stages)
            self.assertNotIn('INSPECT',stages)
            self.assertNotIn('RETURN',stages)

    def test_mission_extensions_are_unsolved_for_bust_and_wreck(self):
        from roscon_stonefish_workshop.autonomy_exercises import (
            choose_next_best_views,discover_target,plan_safe_route
        )
        with self.assertRaises(NotImplementedError):
            choose_next_best_views('bust',None,None,OBSTACLES)
        with self.assertRaises(NotImplementedError):
            discover_target([], 'wreck')
        with self.assertRaises(NotImplementedError):
            plan_safe_route((0,0,7),(5,5,7),OBSTACLES)

    def test_pid_uses_correct_topic_without_duplicate_allocator(self):
        robot=(ROOT/'data/robots/bluerov2_photogrammetry.scn').read_text()
        self.assertIn('topic="/bluerov2/odometry"', robot)
        pid=(ROOT/'roscon_stonefish_workshop/bluerov_pid.py').read_text()
        self.assertIn("'odom_topic':'/bluerov2/odometry'",pid)
        launch=(ROOT/'launch/bluerov2_workshop.launch.py').read_text()
        self.assertNotIn("executable='thruster_adapter'",launch)
        self.assertIn("executable='inspection_reference'",launch)
        self.assertIn("LaunchConfiguration('autonomy_mode')",launch)

    def test_proxy_clearance_flags_unsafe_path_and_false_completion(self):
        safe=obstacle_clearance_proxy([[0,-16.,-6.,6.1],[1,-17.,-5.,7.]])
        self.assertTrue(safe['clearance_proxy_pass'])
        unsafe=obstacle_clearance_proxy([[0,-16.,-6.,7.],[1,-14.,-6.,7.]])
        self.assertFalse(unsafe['clearance_proxy_pass'])
        self.assertIsNone(obstacle_clearance_proxy([]))
        self.assertEqual(image_view_proxy({'target':'cat','events':[]})['inspection_pose_events'],0)

if __name__=='__main__': unittest.main()
