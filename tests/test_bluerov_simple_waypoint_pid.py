"""Regression tests for the deliberately simple waypoint PID baseline."""
import ast
import math
import unittest
from pathlib import Path

from roscon_stonefish_workshop.bluerov_waypoint_math import (
    ALLOCATION, WaypointGains, allocate, waypoint_efforts, wrap_pi)
from roscon_stonefish_workshop.bluerov_waypoint_demo import relative_goal
from roscon_stonefish_workshop.inspection_reference import (
    build_plan, reached_inspection_pose)

ROOT=Path(__file__).resolve().parents[1]


class SimpleWaypointPID(unittest.TestCase):
    def test_allocation_is_exact_user_matrix(self):
        self.assertEqual(ALLOCATION.tolist(), [
            [1,1,1,0],[1,-1,-1,0],[-1,1,-1,0],[-1,-1,1,0],
            [0,0,0,1],[0,0,0,-1],[0,0,0,-1],[0,0,0,1]])
        self.assertEqual(allocate(0.5,0,0,0,limit=.4),[.4,.4,-.4,-.4,0,0,0,0])

    def test_initial_cat_waypoint_turns_without_moving_or_diving(self):
        plan=build_plan((-16,-6,6.1,math.pi),'cat')
        self.assertEqual(plan[0]['phase'],'SEARCH')
        self.assertAlmostEqual(plan[0]['z'],6.1)
        (surge,sway,yaw_cmd,heave),mode,dist,ez,heading_error=waypoint_efforts(
            -16,-6,6.1,math.pi,plan[0]['x'],plan[0]['y'],plan[0]['z'],plan[0]['yaw'],
            turn_depth=6.1)
        self.assertEqual(mode,'TURN')
        self.assertEqual(surge,0)
        self.assertEqual(sway,0)
        self.assertEqual(heave,0)
        self.assertGreater(abs(yaw_cmd),.1)
        self.assertLess(abs(heading_error),math.pi)

    def test_forward_goal_aligned_and_depth_error_commands(self):
        (s,side,y,h),mode,_,_,_=waypoint_efforts(
            0,0,6.1,0,3,0,6.1,0,turn_depth=6.1)
        self.assertEqual(mode,'MOVE')
        self.assertGreater(s,.20)
        self.assertEqual(side,0)
        self.assertAlmostEqual(y,0)
        self.assertAlmostEqual(h,0)
        (s,side,y,h),mode,_,ez,_=waypoint_efforts(
            0,0,6.1,0,3,0,6.6,0)
        self.assertGreater(ez,.4)
        self.assertLess(h,0)  # joystick signs are configurable, initially -1

    def test_approaching_goals_brakes_with_velocity(self):
        base=waypoint_efforts(0,0,7,0,1.5,0,7,0,forward_speed=0)[0][0]
        moving=waypoint_efforts(0,0,7,0,1.5,0,7,0,forward_speed=1.0)[0][0]
        self.assertLess(moving,base)
        self.assertEqual(waypoint_efforts(0,0,7,0,0.1,0,7,0)[1],'SETTLE')

    def test_relative_goal_uses_current_heading(self):
        x,y,z,yaw=relative_goal(-16,-6,6.1,math.pi,forward=2)
        self.assertAlmostEqual(x,-18)
        self.assertAlmostEqual(y,-6)
        self.assertEqual(z,6.1)
        x,y,z,yaw=relative_goal(-16,-6,6.1,math.pi,turn_deg=30,down=.4)
        self.assertAlmostEqual(z,6.5)
        self.assertAlmostEqual(wrap_pi(yaw-math.pi),math.pi/6)

    def test_inspection_pose_requires_heading(self):
        pos=(-6.4,-6,7.1)
        p={'x':pos[0],'y':pos[1],'z':pos[2],'yaw':0.5}
        # quaternion yaw z=.25 w~.9689
        self.assertFalse(reached_inspection_pose(pos,(0,0,0,1),p))
        self.assertTrue(reached_inspection_pose(pos,(0,0,math.sin(.25),math.cos(.25)),p))

    def test_launch_and_readme_use_real_parameter_yaml(self):
        s=(ROOT/'launch/bluerov2_workshop.launch.py').read_text()
        self.assertIn("PathJoinSubstitution([pkg,'config','bluerov_pid.yaml'])",s)
        s=(ROOT/'launch/bluerov_controller.launch.py').read_text()
        self.assertNotIn('thruster_adapter',s)
        conf=(ROOT/'config/bluerov_pid.yaml').read_text()
        self.assertIn('photogrammetry_waypoint_pid:',conf)
        self.assertIn('heave_sign: -1.0',conf)
        self.assertIn('yaw_sign: -1.0',conf)
        self.assertIn('bluerov_waypoint_demo =', (ROOT/'setup.py').read_text())
        self.assertIn('bluerov_waypoint_demo', (ROOT/'README.md').read_text())
        for f in (ROOT/'roscon_stonefish_workshop/bluerov_pid.py',
                  ROOT/'roscon_stonefish_workshop/bluerov_waypoint_demo.py',
                  ROOT/'roscon_stonefish_workshop/bluerov_waypoint_math.py'):
            ast.parse(f.read_text())


if __name__=='__main__':unittest.main()
