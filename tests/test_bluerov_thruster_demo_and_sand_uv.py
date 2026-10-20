"""Check direct 8-thruster familiarisation and seabed texturing, offline."""
import unittest
import hashlib
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

class ROVThrusterDemoAndSandTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=ROOT/'roscon_stonefish_workshop/bluerov_thruster_demo.py'
        spec=importlib.util.spec_from_file_location('bluerov_thruster_demo', path)
        cls.demo=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.demo)

    def test_order_matches_stonefish_robot(self):
        root=ET.parse(ROOT/'data/robots/bluerov2_photogrammetry.scn').getroot()
        order=tuple(x.attrib['name'] for x in root.findall('.//actuator[@type="thruster"]'))
        self.assertEqual(self.demo.THRUSTERS, order)
        self.assertEqual(len(order),8)

    def test_single_motor_and_limits(self):
        for i,name in enumerate(self.demo.THRUSTERS):
            u=self.demo.make_pattern(thruster=name,value=.10)
            self.assertEqual(len(u),8)
            self.assertAlmostEqual(u[i],.10)
            self.assertEqual(sum(v!=0 for v in u),1)
        for x in [-0.3,0.30,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):self.demo.make_pattern(thruster='FrontRight',value=x)

    def test_axis_patterns_and_controller_disabled_default(self):
        self.assertEqual(self.demo.make_pattern(axis='surge',value=.1),[.1,.1,-.1,-.1,0.,0.,0.,0.])
        self.assertEqual(self.demo.make_pattern(axis='heave',value=.1),[0.,0.,0.,0.,.1,-.1,-.1,.1])
        launch=(ROOT/'launch/bluerov2_workshop.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('controller',default_value='false')",launch)
        setup=(ROOT/'setup.py').read_text()
        self.assertIn('bluerov_thruster_demo =',setup)

    def test_sand_unchanged_and_uv_not_stretched(self):
        scene=ET.parse(ROOT/'data/scenarios/bluerov2_photogrammetry.scn').getroot()
        seabed=scene.find('./static[@name="Seabed"]')
        self.assertEqual(seabed.find('look').attrib['uv_scale'],'1.0')
        self.assertEqual(len(scene.findall('./light')),6)
        self.assertFalse(any(x.attrib.get('name','').startswith('SeafloorRock_') for x in scene))
        self.assertEqual(len(ET.parse(ROOT/'data/robots/bluerov2_photogrammetry.scn').getroot().findall('./robot/light')),0)
        raw=(ROOT/'data/photogrammetry/textures/seabed.jpg').read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(),'3d54f5b4ef7d040'+hashlib.sha256(raw).hexdigest()[15:])
        self.assertIn('uv_scale="1.0"',(ROOT/'README.md').read_text())

if __name__=='__main__':unittest.main()
