"""Check genuine, decipherable checkpoint tags and no fake-marker overlay."""
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parents[1]


class ArucoMarkers(unittest.TestCase):
    def test_twelve_paired_aruco_textures(self):
        try:
            import cv2
        except ImportError:
            self.skipTest('cv2 not present on this test computer')
        if not hasattr(cv2, 'aruco'):
            self.skipTest('cv2.aruco not present on this test computer')
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for n in range(1,13):
          for side, filename, expected_id in (
              ('R', f'sign_cp{n:02d}.png', 10+n),
              ('L', f'sign_cp{n:02d}_left.png', 30+n),
          ):
            path = BASE/'data'/'textures'/filename
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            self.assertIsNotNone(image)
            self.assertEqual(tuple(image.shape), (1024,1024))
            if hasattr(cv2.aruco, 'ArucoDetector'):
                _corners, ids, _rejected = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
            else:
                _corners, ids, _rejected = cv2.aruco.detectMarkers(image, dictionary)
            self.assertEqual(ids.flatten().tolist(), [expected_id])
            # The marker MUST remain fully visible when Stonefish maps only
            # the central half of the PNG onto the box face.
            crop = image[256:768, 256:768]
            if hasattr(cv2.aruco, 'ArucoDetector'):
                _c2, ids2, _r2 = cv2.aruco.ArucoDetector(dictionary).detectMarkers(crop)
            else:
                _c2, ids2, _r2 = cv2.aruco.detectMarkers(crop, dictionary)
            self.assertIsNotNone(ids2)
            self.assertEqual(ids2.flatten().tolist(), [expected_id])
            self.assertTrue((crop[:64,:] == 255).all())
            self.assertTrue((crop[-64:,:] == 255).all())
            self.assertTrue((crop[:,:64] == 255).all())
            self.assertTrue((crop[:,-64:] == 255).all())
            self.assertTrue((image[:100,:] == 255).all())
            self.assertTrue((image[-100:,:] == 255).all())
            self.assertTrue((image[:,:100] == 255).all())
            self.assertTrue((image[:,-100:] == 255).all())

    def test_separate_direction_panels_and_non_colliding(self):
        root = ET.parse(BASE/'data'/'scenarios'/'blueboat_race.scn').getroot()
        objects = root.findall('animated')
        marker = [o for o in objects if o.get('name','').startswith('Aruco_Direction_')]
        arrows = [o for o in objects if o.get('name','').startswith('Arrow_Direction_')]
        self.assertEqual(len(marker),24)
        self.assertEqual(len(arrows),24)
        for n in range(1,13):
            for side in ('L','R'):
                self.assertIn(f'Aruco_Direction_CP{n:02d}_{side}',
                              [o.get('name') for o in marker])
                self.assertIn(f'Arrow_Direction_CP{n:02d}_{side}',
                              [o.get('name') for o in arrows])
        self.assertTrue(all(o.get('collisions')=='false' for o in marker+arrows))
        self.assertTrue(all(o.get('type')=='box' for o in marker+arrows))
        self.assertTrue(all(o.find('look').get('uv_mode')=='1' for o in marker+arrows))
        self.assertTrue(all(o.find('dimensions').get('xyz') == '0.035 1.60 1.60' for o in marker))
        self.assertTrue(all(o.find('dimensions').get('xyz') == '0.035 1.60 0.48' for o in arrows))
        names = [o.attrib['name'] for o in objects]
        self.assertEqual(len(names),len(set(names)))

if __name__=='__main__':
    unittest.main()
