"""Offline checks for photograph-based seabed and original BlueROV2 with standard IMU."""
import hashlib
import json
import math
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
DATA=ROOT/'data'

class FeaturedSeabedAndOriginalROV(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene=ET.parse(DATA/'scenarios/bluerov2_photogrammetry.scn').getroot()
        cls.vehicle=ET.parse(DATA/'robots/bluerov2_photogrammetry.scn').getroot()

    def test_original_real_robot_meshes_installed(self):
        self.assertEqual(len(self.vehicle.findall('.//actuator[@type="thruster"]')),8)
        for xml_mesh in self.vehicle.findall('.//mesh'):
            filename=xml_mesh.get('filename')
            self.assertTrue((DATA/filename).is_file(),filename)
        self.assertGreater((DATA/'bluerov2_unphy/bluerov2.obj').stat().st_size,1000000)
        self.assertIn("'data/bluerov2_unphy'", (ROOT/'setup.py').read_text())
        self.assertEqual(self.vehicle.find('.//external_part[@name="HullBottom"]/visual/mesh').get('filename'),'bluerov2_unphy/bluerov2.obj')
        self.assertEqual(len(self.vehicle.findall('./robot')),1)
        self.assertIsNotNone(self.vehicle.find('./robot/link[@name="link_m"]'))
        self.assertIsNotNone(self.vehicle.find('./robot/joint[@name="base_link_to_multibeam"]'))

    def test_single_standard_imu_160(self):
        sensor=self.vehicle.find('.//sensor[@name="IMU"]')
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.get('type'),'imu')
        self.assertEqual(sensor.get('rate'),'100.0')
        self.assertEqual(sensor.find('range').get('angular_velocity'),'10.0 10.0 5.0')
        self.assertEqual(sensor.find('range').get('linear_acceleration'),'10.0')
        self.assertEqual(sensor.find('noise').get('angle'),'0.1 0.1 0.5')
        self.assertEqual(sensor.find('noise').get('angular_velocity'),'0.05')
        self.assertEqual(sensor.find('noise').get('yaw_drift'),'0.001')
        self.assertEqual(sensor.find('noise').get('linear_acceleration'),'0.1')
        self.assertEqual(sensor.find('history').get('samples'),'1')
        self.assertEqual(sensor.find('link').get('name'),'base_link')
        self.assertEqual(sensor.find('ros_publisher').get('topic'),'/bluerov2/imu/data')
        self.assertEqual(len(self.vehicle.findall('.//sensor[@type="imu"]')),1)
        self.assertFalse(list(self.vehicle.iter('imu_model')))

    def test_original_sand_seabed_without_rocks(self):
        texture=DATA/'photogrammetry/textures/seabed.jpg'
        with Image.open(texture) as im:
            self.assertEqual(im.size,(2048,2048))
        report=json.loads((DATA/'photogrammetry/featured_scene_manifest.json').read_text())
        self.assertEqual(hashlib.sha256(texture.read_bytes()).hexdigest(),report['source_texture_sha256'])
        self.assertEqual(report['source_seabed_texture'],'sand.jpg')
        self.assertEqual(report['new_rocks'],0)
        self.assertFalse((DATA/'photogrammetry/rocks').exists())
        self.assertFalse((DATA/'photogrammetry/textures/pebble_source.jpg').exists())
        self.assertFalse(any(e.get('name','').startswith('SeafloorRock_') for e in self.scene))
        terrain=self.scene.find('./static[@name="Seabed"]')
        self.assertEqual(terrain.get('type'),'terrain')
        self.assertEqual(terrain.find('look').get('name'),'Seabed')
        self.assertEqual(terrain.find('look').get('uv_scale'),'1.0')
        self.assertTrue((DATA/terrain.find('height_map').get('filename')).is_file())
        with Image.open(DATA/terrain.find('height_map').get('filename')) as h:
            self.assertEqual(h.mode,'I;16')
        self.assertEqual(self.scene.find('./looks/look[@name="Seabed"]').get('texture'),'photogrammetry/textures/seabed.jpg')
        water=self.scene.find('./environment/ocean/water')
        self.assertEqual(water.get('density'),'1031.0')
        self.assertLess(abs(report['buoyancy_check']['net_buoyancy_equivalent_kg']),0.01)

    def test_restored_terrain_and_removed_vehicle_lights(self):
        with Image.open(DATA/'photogrammetry/terrain_height.png') as h:
            a=__import__('numpy').asarray(h,dtype='float64')/65535
        self.assertGreater((a.max()-a.min())*1.2,0.9)
        self.assertLess((a.max()-a.min())*1.2,1.1)
        self.assertEqual(len(self.vehicle.findall('./robot/light')),0)
        self.assertEqual(len(self.scene.findall('./light')),6)

    def test_topic_and_docs(self):
        self.assertEqual(len(list(ROOT.rglob('README.md'))),1)
        readme=(ROOT/'README.md').read_text()
        self.assertIn('bluerov2_workshop.launch.py',readme)
        self.assertIn('/bluerov2/imu/data',readme)
        self.assertIn('sand.jpg',readme.lower())
        self.assertIn('no generated mosaic texture',readme.lower())

if __name__=='__main__':
    unittest.main()
