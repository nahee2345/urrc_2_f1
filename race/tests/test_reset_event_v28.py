#!/usr/bin/env python3
"""Destructive reset tests use only temporary directories, never the real race."""
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from reset_event_v28 import reset

class ResetEventTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        (self.root/'race/scripts').mkdir(parents=True)
        (self.root/'race/scripts/stage_v3.py').write_text('# test marker\n')
        self.team=self.root/'src/team/team_08/race_entry.json'
        self.team.parent.mkdir(parents=True)
        self.team.write_text('{"entry_id":"team_08"}')
        self.mapasset=self.root/'src/urrc_track_gazebo/models/road.obj'
        self.mapasset.parent.mkdir(parents=True)
        self.mapasset.write_text('road')
        self.old_map=self.root/'race/results/v3_maps.json'
        self.old_map.parent.mkdir(parents=True)
        self.old_map.write_text(json.dumps({'maps':{
            'qualifying':'hungaroring','semi_a':'cota',
            'semi_b':'baku','final':'monaco'}}))
        for name in ('qualifying.json','semi_a.json','semi_b.json',
                     'final.json','finalists_v27.json'):
            (self.old_map.parent/name).write_text(name)
        p=self.root/'race/results/qualifying_individual/manifest.json'
        p.parent.mkdir()
        p.write_text('manifest')
        p=self.root/'race/results/summary_txt'
        p.mkdir()
        (p/'rankings.txt').write_text('rank')
        p=self.root/'race/previews/v17_qualifying_test/report.json'
        p.parent.mkdir(parents=True)
        p.write_text('preview')
        p=self.root/'race/test_reports/test.txt'
        p.parent.mkdir(parents=True)
        p.write_text('report')

    def test_confirmed_reset_removes_generated_data_not_sources(self):
        out=io.StringIO()
        data=reset(self.root,stdin=io.StringIO('RESET\n'),stdout=out)
        self.assertEqual(len(set(data['maps'].values())),4)
        self.assertTrue(self.old_map.is_file())
        self.assertFalse((self.root/'race/results/qualifying.json').exists())
        self.assertFalse((self.root/'race/results/qualifying_individual').exists())
        self.assertFalse((self.root/'race/results/finalists_v27.json').exists())
        self.assertFalse((self.root/'race/previews/v17_qualifying_test').exists())
        self.assertFalse((self.root/'race/test_reports/test.txt').exists())
        self.assertEqual(self.team.read_text(),'{"entry_id":"team_08"}')
        self.assertEqual(self.mapasset.read_text(),'road')
        self.assertIn('RESET',out.getvalue())

    def test_cancellation_keeps_everything(self):
        with self.assertRaisesRegex(ValueError,'취소'):
            reset(self.root,stdin=io.StringIO('NO\n'),stdout=io.StringIO())
        self.assertTrue((self.root/'race/results/qualifying.json').exists())
        self.assertTrue((self.root/'race/previews/v17_qualifying_test/report.json').exists())

    def test_symlink_refused(self):
        linked=self.root/'race/previews/linked'
        linked.symlink_to(self.team)
        with self.assertRaisesRegex(ValueError,'심볼릭'):
            reset(self.root,yes=True,stdout=io.StringIO())
        self.assertTrue(self.old_map.is_file())
        self.assertTrue(self.team.is_file())

if __name__=='__main__':
    unittest.main()
