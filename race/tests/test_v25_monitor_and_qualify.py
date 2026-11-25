import importlib.util
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import ModuleType
import unittest

HERE = Path(__file__).resolve().parents[1] / 'scripts'

def load(name):
    path = HERE / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    obj = importlib.util.module_from_spec(spec)
    sys.modules[name] = obj
    spec.loader.exec_module(obj)
    return obj

stub = ModuleType('race_monitor_v6')
stub.follow_jsonl = lambda *args: None
stub.load_json = lambda path, fallback=None: fallback
stub.format_seconds = lambda t: '-' if t is None else f'{t:.3f}'
sys.modules['race_monitor_v6'] = stub
monitor = load('race_monitor_v25')
qualify = load('qualify_one_v25')

def event(team, kind, **fields):
    return {'type': kind, 'entry_id': team, **fields}

class MonitorTests(unittest.TestCase):
    def test_same_checkpoint_second_gap_and_order(self):
        events = [event('team_01', 'arm'), event('team_02', 'arm'),
                  event('team_01', 'crossing', time=5.0, gate=0),
                  event('team_02', 'crossing', time=5.5, gate=0),
                  event('team_02', 'crossing', time=11.5, gate=1),
                  event('team_01', 'crossing', time=10.0, gate=1)]
        rows = monitor.summarize(events, ['team_01', 'team_02'])
        self.assertEqual([r['team'] for r in rows], ['team_01', 'team_02'])
        self.assertEqual(rows[1]['gap_ahead_s'], 1.5)
        self.assertEqual(rows[1]['gap_leader_s'], 1.5)
        self.assertEqual(rows[0]['gap_leader_s'], 0.0)
    def test_more_checkpoint_progress_wins(self):
        events = [event('team_01', 'crossing', time=4.0, gate=0),
                  event('team_01', 'crossing', time=8.0, gate=1),
                  event('team_01', 'crossing', time=12.0, gate=2),
                  event('team_02', 'crossing', time=5.0, gate=0),
                  event('team_02', 'crossing', time=9.5, gate=1)]
        rows = monitor.summarize(events, ['team_01', 'team_02'])
        self.assertEqual(rows[0]['team'], 'team_01')
        self.assertEqual(rows[1]['gap_ahead_s'], 1.5)
        self.assertEqual(rows[1]['gap_checkpoints'], 1)
    def test_no_crossing_means_no_invented_seconds(self):
        rows = monitor.summarize([], ['team_01', 'team_02'])
        self.assertIsNone(rows[1]['gap_ahead_s'])
    def test_dnf_does_not_lead_active_racer(self):
        events = [event('team_01','crossing',time=4.0,gate=0),
                  event('team_01','crossing',time=8.0,gate=1),
                  event('team_02','crossing',time=5.0,gate=0)]
        rows = monitor.summarize(events,['team_01','team_02'],[{'entry_id':'team_01','status':'DNF','laps':0}])
        self.assertEqual(rows[0]['team'],'team_02')
        self.assertIsNone(rows[1]['gap_ahead_s'])

class QualifyTests(unittest.TestCase):
    def test_one_by_one_accumulates_but_never_confirms_early(self):
        with TemporaryDirectory() as d:
            root = Path(d)
            (root/'race/results/qualifying_individual').mkdir(parents=True)
            (root/'race/scripts').mkdir()
            (root/'src').mkdir()
            oldtiming = sys.modules.get('timing')
            oldmanager = sys.modules.get('race_manager')
            fake_timing = ModuleType('timing')
            def ranks(rows):
                rows = sorted(rows, key=lambda r:r['time_seconds'])
                for i, r in enumerate(rows):r.update(rank=i+1,rank_provisional=False)
                return rows
            fake_timing.rank_rows = ranks
            fake_mgr = ModuleType('race_manager')
            def save_results(res, run_dir, mode, payload):
                for folder in (res,run_dir):
                    folder.mkdir(parents=True, exist_ok=True)
                    (folder/(mode+'.json')).write_text(json.dumps(payload),encoding='utf-8')
            fake_mgr.save_results = save_results
            sys.modules['timing'] = fake_timing
            sys.modules['race_manager'] = fake_mgr
            try:
                sess={'teams':['team_01','team_02'],'event_id':'manual123','track':'monza','fingerprints':{}}
                def result(team,t):
                    return {'valid':True,'state':'complete','track':'monza',
                            'results':[{'entry_id':team,'status':'finished',
                                        'time_seconds':t, 'laps':2,'lap_times_seconds':[t/2,t/2]}]}
                base = root/'race/results/qualifying_individual'
                qualify.atomic_json(base/'team_01.json',result('team_01',34.2))
                partial, done = qualify.accumulate(root,sess)
                self.assertFalse(done)
                self.assertFalse(partial['valid'])
                self.assertEqual(partial['state'],'collecting')
                qualify.atomic_json(base/'team_02.json',result('team_02',33.8))
                merged,done = qualify.accumulate(root,sess)
                self.assertTrue(done)
                self.assertTrue(merged['valid'])
                self.assertEqual(merged['state'],'complete')
                self.assertEqual([r['entry_id'] for r in merged['results']],['team_02','team_01'])
                self.assertTrue((root/'src/fast_lap.txt').is_file())
            finally:
                if oldtiming is None:sys.modules.pop('timing',None)
                else:sys.modules['timing']=oldtiming
                if oldmanager is None:sys.modules.pop('race_manager',None)
                else:sys.modules['race_manager']=oldmanager
    def test_snapshot_locks_entrant_files_and_map(self):
        with TemporaryDirectory() as d:
            root=Path(d)
            for p in ('race','race/results','race/entrants/team_01','race/entrants/team_02'):
                (root/p).mkdir(parents=True,exist_ok=True)
            (root/'race/v24_profile.json').write_text('{"mode":"official"}')
            (root/'race/results/v3_maps.json').write_text('{"maps":{"qualifying":"monza"}}')
            (root/'race/event_config.json').write_text('{}')
            for team in ('team_01','team_02'):
                (root/'race/entrants'/team/'race_entry.json').write_text('{"entry_id":"'+team+'"}')
            s=qualify.snapshot(root)
            self.assertEqual(s['teams'],['team_01','team_02'])
            self.assertEqual(s['track'],'monza')
            self.assertEqual(len(s['fingerprints']),2)

if __name__=='__main__':unittest.main()
