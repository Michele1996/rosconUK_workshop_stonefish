"""Offline regression checks for dive authority and wrong-sign yaw guard."""
import math
import unittest
from roscon_stonefish_workshop.bluerov_waypoint_math import (
    YawDivergenceGuard, WaypointGains, waypoint_efforts, allocate,
)


class BlueROVDiveYawFix(unittest.TestCase):
    def test_depth_reaches_half_scale_at_40cm(self):
        gains=WaypointGains()
        self.assertAlmostEqual(gains.max_heave, .5)
        cmd, mode, *_ = waypoint_efforts(
            0, 0, 6.1, 0, 0, 0, 6.5, 0, gains=gains)
        self.assertEqual(mode, 'SETTLE')
        self.assertAlmostEqual(cmd[3], gains.heave_sign*.48, places=5)
        self.assertLessEqual(max(map(abs, allocate(*cmd))), .65)

    def test_depth_saturates_at_half_scale(self):
        cmd, *_ = waypoint_efforts(0, 0, 6, 0, 0, 0, 6.6, 0)
        self.assertAlmostEqual(abs(cmd[3]), .5)

    def test_yaw_reversal_on_sustained_wrong_direction(self):
        guard=YawDivergenceGuard(-1)
        # A positive NED yaw error gets a negative motor command for -1
        # polarity. If measured yaw rate is negative, error keeps growing.
        events=[]
        for i in range(20):
            events.append(guard.update(
                i*.05, 'TURN', math.radians(35), -.08, -.18))
        self.assertEqual(events.count('REVERSED'), 1)
        self.assertTrue(guard.reversed_once)
        self.assertEqual(guard.sign, 1)
        self.assertFalse(guard.locked)

    def test_no_false_flip_in_motion_or_when_turning_correctly(self):
        for mode, rate, motor in [('MOVE', -.15, -.22),
                                  ('TURN', .15, -.22),
                                  ('SETTLE', -.15, .22)]:
            guard=YawDivergenceGuard(-1)
            for i in range(40):
                guard.update(i*.05, mode, math.radians(35), rate, motor)
            self.assertFalse(guard.reversed_once)
            self.assertFalse(guard.locked)

    def test_yaw_locks_if_both_polarities_diverge(self):
        guard=YawDivergenceGuard(-1)
        for i in range(14):
            guard.update(i*.05,'TURN',.5,-.15,-.2)
        self.assertEqual(guard.sign,1)
        self.assertFalse(guard.locked)
        events=[]
        # Emulate a period after reverse transients have settled.
        for i in range(50):
            t=2.0+i*.05
            events.append(guard.update(t, 'TURN', .5, -.15, .2))
        self.assertEqual(events.count('LOCKED'),1)
        self.assertTrue(guard.locked)

    def test_rate_braking_inside_heading_deadband(self):
        cmd, *_ = waypoint_efforts(
            0,0,6,0,0,0,6,math.radians(1),yaw_rate=0.30)
        # Opposes the measured yaw rate in NED convention.
        self.assertLess(cmd[2]*WaypointGains().yaw_sign,0)


if __name__ == '__main__':
    unittest.main()
