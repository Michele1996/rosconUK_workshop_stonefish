"""Offline polarity regression: positive Twist surge must mean forward."""
import ast
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / 'roscon_stonefish_workshop' / 'blueboat_thruster_adapter.py'


def functions():
    tree = ast.parse(ADAPTER.read_text())
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and
             node.name in ('mix_thrusters', 'map_twist_to_setpoints')]
    ns = {'math': math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(ADAPTER), 'exec'), ns)
    return ns['map_twist_to_setpoints']


class PolarityTests(unittest.TestCase):
    def test_positive_surge_goes_forward(self):
        map_setpoints = functions()
        self.assertEqual(map_setpoints(0.15, 0.0), [0.15, 0.15])
        self.assertEqual(map_setpoints(-0.15, 0.0), [-0.15, -0.15])
        self.assertEqual(map_setpoints(0.0, 0.0), [0.0, 0.0])

    def test_yaw_mix_unchanged(self):
        map_setpoints = functions()
        # Earlier release negated mix_thrusters without reversing yaw input.
        self.assertAlmostEqual(map_setpoints(0.0, 0.2)[0], +0.2)
        self.assertAlmostEqual(map_setpoints(0.0, 0.2)[1], -0.2)
        self.assertAlmostEqual(map_setpoints(0.0, -0.2)[0], -0.2)
        self.assertAlmostEqual(map_setpoints(0.0, -0.2)[1], +0.2)

    def test_sign_override_restores_old_mapping(self):
        map_setpoints = functions()
        self.assertEqual(map_setpoints(0.15, 0.0, surge_sign=1.0), [-0.15, -0.15])
        with self.assertRaises(ValueError):
            map_setpoints(0.15, 0.0, surge_sign=0.0)

    def test_mixing_is_bounded(self):
        map_setpoints = functions()
        for s in (-1, -0.5, 0, 0.5, 1):
            for y in (-1, 0, 1):
                self.assertTrue(all(abs(x) <= 1 for x in map_setpoints(s, y)))


if __name__ == '__main__':
    unittest.main()
