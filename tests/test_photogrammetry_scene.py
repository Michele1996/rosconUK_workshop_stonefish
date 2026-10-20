"""Offline asset/XML checks; not a substitute for launching Stonefish 1.6."""
from pathlib import Path
from xml.etree import ElementTree as ET
import unittest
from PIL import Image

PKG = Path(__file__).resolve().parents[1]


class Photogrammetry(unittest.TestCase):
    def setUp(self):
        self.scene = ET.parse(PKG/'data/scenarios/bluerov2_photogrammetry.scn').getroot()
        self.robot = ET.parse(PKG/'data/robots/bluerov2_photogrammetry.scn').getroot()

    def test_real_terrain_and_texture(self):
        terrain=self.scene.find('./static[@name="Seabed"]')
        self.assertEqual(terrain.get('type'), 'terrain')
        self.assertEqual(terrain.find('./dimensions').get('height'), '1.2')
        hp=PKG/'data'/terrain.find('./height_map').get('filename')
        with Image.open(hp) as im:
            self.assertEqual(im.size, (512,512))
            self.assertIn(im.mode, ('I;16','I'))
        looks={x.get('name'): x for x in self.scene.findall('./looks/look')}
        tex=PKG/'data'/looks['Seabed'].get('texture')
        self.assertTrue(tex.is_file())

    def test_three_target_assets_and_resolved_meshes(self):
        self.assertEqual(len([e for e in self.scene.findall('./static[@type="model"]') if e.get('name') in ('cat_target_0','bust_target_0','wr_target_0')]),3)
        self.assertGreaterEqual(len(self.scene.findall('./animated[@type="model"]')),10)
        for r in (self.scene,self.robot):
            for mesh in r.iter('mesh'):
                self.assertTrue((PKG/'data'/mesh.get('filename')).is_file(), mesh.attrib)

    def test_robot_actuators_sensors_and_looks(self):
        self.assertEqual(len(self.robot.findall('.//actuator[@type="thruster"]')),8)
        self.assertEqual(len(self.robot.findall('.//sensor[@type="camera"]')),1)
        mats={x.get('name') for x in self.scene.findall('./materials/material')}
        looks={x.get('name') for x in self.scene.findall('./looks/look')}
        for r in (self.robot,self.scene):
            for m in r.iter('material'):
                self.assertIn(m.get('name'),mats)
            for m in r.iter('look'):
                self.assertIn(m.get('name'),looks)

    def test_capture_readme_and_single_readme(self):
        self.assertTrue((PKG/'roscon_stonefish_workshop/photogrammetry_capture.py').exists())
        self.assertTrue((PKG/'scripts/run_photogrammetry_colmap.sh').exists())
        self.assertEqual(len(list(PKG.rglob('README.md'))),1)
        self.assertIn('bluerov2_workshop.launch.py',(PKG/'README.md').read_text())
