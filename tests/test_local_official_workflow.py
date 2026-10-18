"""Offline persistence + decentralized export tests; no ROS needed."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from zipfile import ZipFile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from roscon_stonefish_workshop.race_records import AttemptStore, sha256_file
from roscon_stonefish_workshop.race_export import export
from roscon_stonefish_workshop.race_engine import RaceEngine
from roscon_stonefish_workshop.race_geometry import load_course, sample_course, length


class LocalWorkflow(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.course_path=ROOT/'config/blueboat_race.json'
        self.sha=sha256_file(self.course_path)
        self.course=load_course(self.course_path)

    def tearDown(self): self.temp.cleanup()

    def store(self): return AttemptStore(self.root,'Team 01',self.course['course_version'],self.sha)

    def test_three_consumed_reboot_persistence(self):
        store=self.store()
        for attempt in range(3):
            entry=store.start(0.)
            self.assertEqual(entry['attempt_number'],attempt+1)
            self.assertEqual(self.store().remaining,2-attempt) # re-open as if simulator restarted
            store.finalize('DNF_CANCELLED')
        self.assertEqual(self.store().remaining,0)
        with self.assertRaises(RuntimeError): self.store().start(0.)

    def test_finished_run_and_best_export(self):
        store=self.store()
        engine=RaceEngine(self.course)
        engine.disarm()
        for i in range(100):
            p=sample_course(self.course, length(self.course)*i/99)[0]
            practice=engine.update(p,float(i)/4)
        self.assertEqual(practice['state'],'practice')
        self.assertEqual(store.consumed,0)
        store.start(100.0)
        engine.arm()
        for i in range(1501):
            p=sample_course(self.course,length(self.course)*i/1500)[0]
            status=engine.update(p,100+i*0.15)
        self.assertEqual(status['state'],'finished')
        store.checkpoint('START',engine.start_time)
        store.trace(120.,*p,status)
        store.finalize('FINISHED',status)
        self.assertEqual(store.remaining,2)
        out,summary,digest=export('Team 01',self.root,self.root/'submitted',self.sha)
        self.assertEqual(len(digest),64)
        self.assertAlmostEqual(summary['best_adjusted_time_s'],status['adjusted_time_s'])
        with ZipFile(out) as z:
            self.assertIn('submission_summary.json',z.namelist())
            self.assertIn('attempt_01/report.json',z.namelist())
            self.assertIn('attempt_01/trajectory.csv',z.namelist())

    def test_practice_starts_without_official_score(self):
        engine=RaceEngine(self.course)
        engine.disarm()
        engine.update((-2,0),0)
        s=engine.update((4,0),1)
        self.assertEqual(s['state'],'practice')
        self.assertEqual(s['race_time_s'],0)
        self.assertFalse(s['race_started'])

    def test_no_map_in_student_folder_and_single_readme(self):
        self.assertFalse((ROOT/'data/blueboat_race_map.png').exists())
        self.assertFalse((ROOT/'data/blueboat_waypoints.csv').exists())
        self.assertEqual([f.name for f in ROOT.rglob('*.md')],['README.md'])

if __name__=='__main__': unittest.main()
