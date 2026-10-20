"""Offline coverage score, geometry metric, guided inspection and integration checks."""
import csv
import json
import tempfile
import unittest
from pathlib import Path
from math import cos,sin,pi
from xml.etree import ElementTree as ET

from roscon_stonefish_workshop.survey_evaluation import (evaluate,read_manifest,statue_score,wreck_score)
from roscon_stonefish_workshop.inspection_mission import inspection_path,reached,TARGETS
from roscon_stonefish_workshop.inspection_evaluation import evaluate as eval_mission
from roscon_stonefish_workshop.photogrammetry_geometry_eval import compute_metrics

ROOT=Path(__file__).resolve().parents[1]


def row(x,y,z=7.,index=0):
    return {'image':f'image_{index:05d}.png','xyz':(x,y,z),'time':float(index)}


class SurveyAndInspection(unittest.TestCase):
    def test_empty_survey_scores_zero_and_separate(self):
        a=evaluate([])
        self.assertEqual(a['statues']['combined_score_out_of_100'],0)
        self.assertEqual(a['shipwreck']['score_out_of_100'],0)
        self.assertNotIn('shipwreck',evaluate([],'statues'))
        self.assertNotIn('statues',evaluate([],'wreck'))

    def test_statue_two_heights_full_azimuth(self):
        rows=[]
        for cx,cy in TARGETS.values():
            if cx>0:continue
            for z in (6.4,8.2):
                for i in range(12):
                    a=2*pi*(i+.5)/12
                    rows.append(row(cx+3*cos(a),cy+3*sin(a),z,len(rows)))
        report=evaluate(rows,'statues')
        self.assertEqual(report['statues']['cat']['score_out_of_100'],100)
        self.assertEqual(report['statues']['bust']['score_out_of_100'],100)
        self.assertEqual(report['statues']['combined_score_out_of_100'],100)

    def test_wreck_multilane(self):
        rows=[]
        for z in (6.5,8.0):
            for x in (14.5,19.,23.5):
                for i in range(6):
                    y=-3.3+(i+.5)*(18.6/6)
                    rows.append(row(x,y,z,len(rows)))
                    rows.append(row(x,y+0.01,z,len(rows)))
        w=wreck_score(rows)
        self.assertEqual(w['viewpoint_cells_visited'],18)
        self.assertEqual(w['score_out_of_100'],100)

    def test_manifest_only_existing_images_and_valid_pose(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'images').mkdir()
            (p/'images'/'image_00000.png').write_bytes(b'png bytes')
            with (p/'manifest.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['image','stamp_sec','stamp_nanosec','odom_x_m','odom_y_m','odom_z_m'])
                w.writerow(['image_00000.png','10','0','-7','-6','7'])
                w.writerow(['image_00001.png','11','0','-7','-6','7'])
                w.writerow(['image_00000.png','12','0','','-6','7'])
            self.assertEqual(len(read_manifest(p)),1)

    def test_paths_have_inspection_and_return(self):
        path=inspection_path('cat',(-16.,-6.,6.1,3.14))
        self.assertTrue(path[0]['phase']=='SEARCH')
        self.assertTrue(any(g['phase']=='APPROACH' for g in path))
        self.assertGreaterEqual(sum(g['phase']=='INSPECT' for g in path),24)
        self.assertEqual(path[-1]['phase'],'RETURN')
        self.assertTrue(reached((-16.,-6.,6.1),path[-1]))
        for k in ('bust','wreck'):
            with self.assertRaises(ValueError): inspection_path(k,(-16.,-6.,6.1,3.14))
        self.assertFalse(reached((0.,0.,0.),{'x':4.,'y':0.,'z':0.}))

    def test_inspection_report_requires_reached_goals(self):
        m={'schema':'roscon-inspection-v1','target':'cat','status':'FINISHED',
           'waypoints_total':30,'waypoints_reached':30,'inspection_waypoints_total':24,
           'inspection_waypoints_reached':24,'returned_home':True,
           'mission_elapsed_wall_s':200.,'events':[{'phase':'APPROACH'}]}
        a={'statues':{'cat':{'score_out_of_100':100}}}
        self.assertEqual(eval_mission(m,a)['score_out_of_100'],100)
        m['returned_home']=False;m['status']='TIMEOUT'
        self.assertLess(eval_mission(m,a)['score_out_of_100'],100)

    def test_cloud_distance_metrics(self):
        import numpy as np
        a=np.asarray([[0,0,0],[1,0,0],[0,1,0]],dtype=float)
        v=compute_metrics(a,a,.1)
        self.assertEqual(v['f_score'],1.)
        v=compute_metrics(a,a+20,.1)
        self.assertEqual(v['f_score'],0.)

    def test_install_entrypoints_and_launcher(self):
        s=(ROOT/'setup.py').read_text()
        for name in ('photogrammetry_evaluate','photogrammetry_geometry_eval','inspection_mission','inspection_evaluate'):
            self.assertIn(name+' =',s)
        launch=(ROOT/'launch/bluerov2_workshop.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('autonomy',default_value='false')",launch)
        self.assertIn("DeclareLaunchArgument('target',default_value='cat')",launch)
        robot=ET.parse(ROOT/'data/robots/bluerov2_photogrammetry.scn').getroot()
        self.assertEqual(len(robot.findall('.//actuator[@type="thruster"]')),8)
        self.assertEqual(len(robot.findall('.//sensor[@type="imu"]')),1)
        self.assertEqual(len(list(ROOT.rglob('README.md'))),1)

    def test_default_value_is_visible_thruster_power(self):
        from roscon_stonefish_workshop.bluerov_thruster_demo import make_pattern
        self.assertAlmostEqual(make_pattern(axis='surge')[0],.25)
        self.assertAlmostEqual(make_pattern(thruster='FrontRight')[0],.25)

if __name__=='__main__':unittest.main()

class Alignment(unittest.TestCase):
    def test_umeyama_known_scale(self):
        import numpy as np
        from roscon_stonefish_workshop.photogrammetry_align import umeyama,fit_alignment
        a=np.random.default_rng(3).normal(size=(18,3))
        R=np.array([[0.,-1,0],[1,0,0],[0,0,1.]])
        b=2.4*(a@R.T)+np.array([10.,5.,-2.])
        scale,gotR,t=umeyama(a,b)
        self.assertAlmostEqual(scale,2.4)
        self.assertTrue(np.allclose(gotR,R,atol=1e-8))
        H,stats=fit_alignment(a,b)
        self.assertEqual(stats['inliers'],len(a))
        self.assertTrue(np.allclose((a@H[:3,:3].T)+H[:3,3],b,atol=1e-5))
