"""Syntax / isolation checks for the Stonefish 1.6 BlueBoat model."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import unittest

P = Path(__file__).resolve().parents[1]

class Stonefish16Diagnostics(unittest.TestCase):
    def test_boat_thruster_definitions(self):
        root = ET.parse(P/'data/robots/blueboat.scn').getroot()
        actuators = root.findall('robot/actuator')
        self.assertEqual([a.get('name') for a in actuators], ['Thruster_R', 'Thruster_L'])
        for act in actuators:
            specs=act.find('specs')
            self.assertAlmostEqual(float(specs.get('max_setpoint')), 366.5)
            self.assertNotIn('max_rpm', specs.attrib)
            self.assertNotIn('thrust_coeff', specs.attrib)
            self.assertEqual(specs.get('normalized_setpoint'), 'true')
            self.assertEqual(act.find('rotor_dynamics').get('type'), 'first_order')
            self.assertEqual(float(act.find('rotor_dynamics/time_constant').get('value')), 0.2)
            self.assertEqual(act.find('thrust_model').get('type'), 'fluid_dynamics')
            self.assertIsNotNone(act.find('thrust_model/thrust_coeff'))
            self.assertIsNotNone(act.find('thrust_model/torque_coeff'))
            self.assertEqual(act.find('link').get('name'), 'Vehicle')

    def test_diagnostic_variants_are_isolated_and_parse(self):
        cases = [
           ('blueboat_only.scn','blueboat.scn',2,6),
           ('blueboat_no_sensors.scn','blueboat_diagnostic_no_sensors.scn',2,0),
           ('blueboat_no_thrusters.scn','blueboat_diagnostic_no_thrusters.scn',0,6),
           ('blueboat_hull_only.scn','blueboat_diagnostic_hull_only.scn',0,0),
        ]
        for scene, robot, n_thrusters, n_sensors in cases:
            with self.subTest(scene=scene):
                s=ET.parse(P/'data/scenarios'/scene).getroot()
                self.assertIsNotNone(s.find('environment/ocean'))
                self.assertEqual(len(s.findall('animated')),0)
                self.assertTrue(s.find('include').get('file').endswith('/'+robot))
                vehicle=ET.parse(P/'data/robots'/robot).getroot().find('robot')
                self.assertEqual(len(vehicle.findall('actuator')),n_thrusters)
                self.assertEqual(len(vehicle.findall('sensor')),n_sensors)
                self.assertEqual(len(vehicle.findall('base_link/external_part')),8)
                self.assertEqual(len(vehicle.findall('base_link/internal_part')),2)

if __name__=='__main__':
    unittest.main()
