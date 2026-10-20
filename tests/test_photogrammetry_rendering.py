"""Regression checks for textured photogrammetry meshes and lighting."""
from pathlib import Path
from xml.etree import ElementTree as ET
from PIL import Image
import unittest
import math

PKG = Path(__file__).resolve().parents[1]
DATA = PKG / 'data'

class PhotogrammetryRendering(unittest.TestCase):
    def test_visual_meshes_have_vertex_normals_and_uvs(self):
        scn=ET.parse(DATA/'scenarios/bluerov2_photogrammetry.scn').getroot()
        targets=[e for e in scn if e.tag in ('static','animated') and e.get('type')=='model']
        self.assertEqual(len(targets),17)
        for target in targets:
            visual=target.find('./visual/mesh')
            self.assertIsNotNone(visual)
            p=DATA/visual.get('filename')
            count={'v':0,'vt':0,'vn':0,'f':0}
            with p.open() as f:
                for l in f:
                    tag=l.split(' ',1)[0]
                    if tag in count:
                        count[tag]+=1
                        if tag=='f':
                            for tok in l.split()[1:]:
                                refs=tok.split('/')
                                self.assertEqual(len(refs),3,(p,tok))
                                self.assertTrue(all(x and int(x)>0 for x in refs))
            self.assertGreater(count['f'],0,p)
            self.assertEqual(count['v'],count['vt'],p)
            self.assertEqual(count['v'],count['vn'],p)
    def test_all_textures_load_and_nonblack(self):
        scn=ET.parse(DATA/'scenarios/bluerov2_photogrammetry.scn').getroot()
        for look in scn.findall('./looks/look'):
            path=look.get('texture')
            if not path:continue
            p=DATA/path
            self.assertTrue(p.is_file(),p)
            with Image.open(p) as im:
                self.assertGreater(im.width,128,p)
                self.assertGreater(im.height,128,p)
                self.assertGreater(sum(im.resize((1,1)).convert('RGB').getpixel((0,0))),40,p)
    def test_targets_are_illuminated_but_vehicle_lamps_absent(self):
        scn=ET.parse(DATA/'scenarios/bluerov2_photogrammetry.scn').getroot()
        robot=ET.parse(DATA/'robots/bluerov2_photogrammetry.scn').getroot()
        lights=scn.findall('./light')
        self.assertGreaterEqual(len(lights),3)
        self.assertTrue(all(float(x.find('specs').get('illuminance'))>0 for x in lights))
        lamps=robot.findall('.//light')
        self.assertEqual(len(lamps),0)

if __name__=='__main__':unittest.main()
