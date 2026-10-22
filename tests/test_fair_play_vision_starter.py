"""Offline checks for local-scoring fair-play controls and camera starter.

No ROS installation is needed: sensor callbacks require a ROS 2 + Stonefish run.
"""
import ast
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from roscon_stonefish_workshop.fair_play import (
    FairPlayViolation, reject_route_file, guard_participant_route_parameters,
    reject_privileged_topic)
from roscon_stonefish_workshop.race_geometry import load_course


class DummyParam:
    def __init__(self, value):
        self.value = value


class DummyNode:
    def __init__(self, values):
        self.values = values
        self.declared = []

    def declare_parameter(self, key, default):
        self.declared.append(key)

    def get_parameter(self, key):
        return DummyParam(self.values.get(key, ''))


class FairPlayVisionStarterTests(unittest.TestCase):
    def test_named_and_renamed_routes_are_rejected_through_supplied_interface(self):
        for path in ('config/blueboat_race.json', '/tmp/copy.json',
                     '/tmp/waypoints.csv', '/tmp/route.yaml'):
            out = StringIO()
            with redirect_stderr(out), self.assertRaises(FairPlayViolation):
                reject_route_file(path, parameter='route_file')
            self.assertIn('FAIR PLAY VIOLATION', out.getvalue())
            self.assertIn('LOCAL RACE JUDGE', out.getvalue())
            self.assertIn(path, out.getvalue())

    def test_judge_topic_rejected_as_participant_observation(self):
        reject_privileged_topic('/blueboat/odom')
        with redirect_stderr(StringIO()), self.assertRaises(FairPlayViolation):
            reject_privileged_topic('/blueboat/race/ground_truth_odom')

    def test_node_route_arguments_and_empty_args(self):
        guard_participant_route_parameters(DummyNode({}))
        for key in ('course_file', 'waypoints_file', 'route_file', 'track_file'):
            with redirect_stderr(StringIO()), self.assertRaises(FairPlayViolation):
                guard_participant_route_parameters(DummyNode({key: '/tmp/renamed.bin'}))

    def test_normal_participant_course_loader_blocked_but_offline_test_allowed(self):
        path = ROOT / 'config' / 'blueboat_race.json'
        with redirect_stderr(StringIO()), self.assertRaises(FairPlayViolation):
            load_course(path)
        self.assertIn('centerline_xy_m', load_course(path, purpose='offline_test'))

    def test_legacy_instructor_solution_aborts(self):
        proc = subprocess.run([sys.executable, str(ROOT/'scripts/blueboat_pid_solution.py')],
                              cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(proc.returncode, 2)
        self.assertIn('FAIR PLAY VIOLATION', proc.stderr)

    def test_student_pid_and_status_have_no_navigation_oracle(self):
        pid = (ROOT/'roscon_stonefish_workshop/blueboat_pid.py').read_text()
        self.assertNotIn('nearest_centerline', pid)
        self.assertNotIn('sample_course', pid)
        self.assertNotIn('load_course', pid)
        mon = (ROOT/'roscon_stonefish_workshop/blueboat_race_monitor.py').read_text()
        self.assertIn("'distance_from_center_m'", mon)
        self.assertIn('json.dumps(public_status', mon)
        self.assertIn('purpose=\'judge\'', mon)

    def test_vision_helpers_on_synthetic_live_frame(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('OpenCV/numpy unavailable')
        src = (ROOT/'roscon_stonefish_workshop/blueboat_camera_steering.py').read_text()
        tree = ast.parse(src)
        target = ['yellow_buoy_candidates', 'initial_gate_target', 'pixel_steering_command']
        funcs = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in target]
        self.assertEqual(len(funcs), 3)
        namespace = {'cv2': cv2, 'np': np, 'clamp': lambda x, a, b: max(a,min(b,x))}
        exec(compile(ast.Module(body=funcs, type_ignores=[]), '<pure_vision>', 'exec'), namespace)
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(frame, (80, 140), 12, (0, 255, 255), -1)
        cv2.circle(frame, (230, 140), 12, (0, 255, 255), -1)
        blobs = namespace['yellow_buoy_candidates'](frame)
        self.assertEqual(len(blobs), 2)
        center = namespace['initial_gate_target'](blobs, 320)
        self.assertAlmostEqual(center, 155.0, delta=2.0)
        self.assertLess(namespace['pixel_steering_command'](center,320,.5,.35), 0)
        self.assertIsNone(namespace['initial_gate_target'](blobs[:1],320))
        self.assertEqual(namespace['pixel_steering_command'](None,320,.5,.35),0)


if __name__ == '__main__':
    unittest.main()
