"""Ensure ROSCon uses the uploaded OBJ boat, not the old substitute hull."""
import math
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

P = Path(__file__).resolve().parents[1]


class UploadedBlueBoatIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.xml = ET.parse(P / 'data/robots/blueboat.scn').getroot()
        self.robot = self.xml.find('robot')

    def test_exact_mesh_hulls_and_real_bluerov_propellers(self):
        parts = {p.attrib['name']: p for p in self.robot.findall('.//external_part')}
        required = {
            'SideR': 'right_side_or.obj',
            'SideL': 'left_side_or.obj',
            'RightBack': 'right_rear_or.obj',
            'LeftBack': 'left_rear_or.obj'
        }
        for name, file in required.items():
            self.assertEqual(parts[name].get('type'), 'model')
            self.assertEqual(parts[name].find('physical/mesh').get('filename'),
                             'opti_blueboat/' + file)
        acts = {a.get('name'): a for a in self.robot.findall('actuator')}
        self.assertEqual(set(acts), {'Thruster_R','Thruster_L'})
        self.assertEqual(acts['Thruster_R'].find('propeller/mesh').get('filename'),
                         'meshes/real_bluerov_cw.obj')
        self.assertEqual(acts['Thruster_L'].find('propeller/mesh').get('filename'),
                         'meshes/real_bluerov_ccw.obj')
        self.assertEqual(acts['Thruster_R'].find('propeller').get('right'), 'false')
        self.assertEqual(acts['Thruster_L'].find('propeller').get('right'), 'true')

    def test_mesh_assets_resolve_in_stonefish_simulation_data(self):
        meshes = self.robot.findall('.//mesh')
        self.assertEqual(len(meshes), 6)
        for m in meshes:
            self.assertTrue((P/'data'/m.get('filename')).is_file(), m.get('filename'))
        self.assertTrue((P/'data/robots/blueboat_original_uploaded.scn').is_file())
        self.assertEqual(len(list((P/'data/opti_blueboat').iterdir())), 47)

    def test_bluerov_dynamics_with_original_geometry_and_topics(self):
        parts = self.robot.findall('.//external_part')
        self.assertEqual(len(parts), 8)
        self.assertEqual(len(self.robot.findall('.//internal_part')), 2)
        for a in self.robot.findall('actuator'):
            sp=a.find('specs')
            self.assertEqual(sp.get('normalized_setpoint'), 'true')
            self.assertAlmostEqual(float(sp.get('max_setpoint')),
                                   366.5, places=7)
            self.assertEqual(a.find('rotor_dynamics').get('type'), 'first_order')
            self.assertEqual(a.find('rotor_dynamics/time_constant').get('value'), '0.2')
            self.assertEqual(a.find('thrust_model').get('type'), 'fluid_dynamics')
            self.assertEqual(a.find('thrust_model/thrust_coeff').get('forward'), '0.07')
        sensors={s.get('name'):s for s in self.robot.findall('sensor')}
        self.assertEqual(set(sensors), {'dvl','gps','imu_filter','race_truth','navigation_odom','bow_camera'})
        self.assertEqual(sensors['dvl'].find('ros_publisher').get('topic'),
                         '/blueboat/navigator/dvl_sim')
        self.assertEqual(sensors['imu_filter'].find('ros_publisher').get('topic'),
                         '/blueboat/navigator/imu')
        self.assertEqual(sensors['gps'].find('ros_publisher').get('topic'),
                         '/blueboat/navigator/gps')
        self.assertEqual(sensors['race_truth'].find('ros_publisher').get('topic'),
                         '/blueboat/race/ground_truth_odom')

    def test_scenario_loads_exact_robot_and_no_collision_buoys(self):
        scene=ET.parse(P/'data/scenarios/blueboat_race.scn').getroot()
        includes=[n.get('file') for n in scene.findall('include')]
        self.assertEqual(includes, ['$(find roscon_stonefish_workshop)/data/robots/blueboat.scn'])
        buoys=[a for a in scene.findall('animated') if a.get('name','').startswith('Buoy_')]
        self.assertGreater(len(buoys),40)
        self.assertTrue(all(a.get('collisions')=='false' for a in buoys))
        markers=[a for a in scene.findall('animated') if a.get('name','').startswith('Aruco_Direction_')]
        self.assertEqual(len(markers),24)


if __name__=='__main__':
    unittest.main()
