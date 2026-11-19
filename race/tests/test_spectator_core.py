"""No ROS/Gazebo required: validate network read-only, DNF fallback and session isolation."""
import http.client
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.parse import quote

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from spectator import RaceData, Handler, WebServer, active_teams, rank_active


class SpectatorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        (self.root/'race/results/runs/example').mkdir(parents=True)
        (self.root/'race/v24_profile.json').write_text('{"mode":"official"}',encoding='utf-8')
        (self.root/'race/results/operator_v26_active.json').write_text(json.dumps({
            'pid':os.getpid(),'stage':'semi_a','track':'monza',
            'teams':['team_01','team_02','team_03'],
            'status':{'team_01':'RUNNING','team_02':'RUNNING','team_03':'RUNNING'}
        }),encoding='utf-8')
        (self.root/'race/results/semi_a.json').write_text(json.dumps({
            'state':'running','audit_directory':'race/results/runs/example','results':[]
        }),encoding='utf-8')
        (self.root/'race/results/runs/example/timing.jsonl').write_text('\n'.join([
            json.dumps({'type':'arm','entry_id':'team_01'}),
            json.dumps({'type':'arm','entry_id':'team_02'}),
            json.dumps({'type':'arm','entry_id':'team_03'}),
            json.dumps({'type':'pose','entry_id':'team_01','sample':[2.,1.,1.,0.1]}),
            json.dumps({'type':'pose','entry_id':'team_02','sample':[2.,2.,1.,0.1]}),
            json.dumps({'type':'pose','entry_id':'team_03','sample':[2.,3.,1.,0.1]}),
            json.dumps({'type':'crossing','entry_id':'team_02','time':1.4,'gate':0}),
            json.dumps({'type':'crossing','entry_id':'team_01','time':1.8,'gate':0}),
        ])+'\n',encoding='utf-8')

    def tearDown(self):self.tmp.cleanup()

    def test_rank_and_fallback_when_dnf(self):
        feed=RaceData(self.root)
        first=feed.state()
        self.assertEqual(first['leader'],'team_02')
        self.assertEqual(len([t for t in first['teams'] if t['available']]),3)
        p=self.root/'race/results/operator_v26_active.json'
        obj=json.loads(p.read_text());obj['status']['team_02']='DNF'
        p.write_text(json.dumps(obj),encoding='utf-8')
        second=feed.state()
        self.assertEqual(second['leader'],'team_01')
        self.assertFalse(next(t for t in second['teams'] if t['team']=='team_02')['available'])
        obj['status']['team_01']='DNF';obj['status']['team_03']='DNF'
        p.write_text(json.dumps(obj),encoding='utf-8')
        self.assertIsNone(feed.state()['leader'])

    def test_only_spawned_teams_followable(self):
        rows=active_teams({'teams':['team_01','team_02','team_03'],
                          'status':{'team_01':'AWAITING_SPAWN','team_02':'SPAWNED','team_03':'FINISHED'}},
                         {'team_01':{},'team_02':{},'team_03':{}})
        self.assertEqual(rows,['team_02'])

    def test_rank_by_crossings(self):
        ev=[{'type':'crossing','entry_id':'team_02','gate':0,'time':4},
            {'type':'crossing','entry_id':'team_01','gate':0,'time':3},
            {'type':'crossing','entry_id':'team_02','gate':1,'time':8}]
        self.assertEqual(rank_active(['team_01','team_02'],ev,[])[0]['team'],'team_02')

    def test_reject_audit_outside_runs(self):
        p=self.root/'race/results/semi_a.json'
        p.write_text(json.dumps({'audit_directory':'../../etc','results':[]}),encoding='utf-8')
        output=RaceData(self.root).state()
        self.assertIsNone(output['leader'])
        self.assertEqual(output['state'],'waiting_vehicles')

    def test_http_get_only_authenticated(self):
        srv=WebServer(('127.0.0.1',0),Handler,self.root,'testing_super_secret')
        t=threading.Thread(target=srv.serve_forever,daemon=True);t.start()
        try:
            conn=http.client.HTTPConnection('127.0.0.1',srv.server_address[1],timeout=3)
            conn.request('GET','/api/state')
            r=conn.getresponse();self.assertEqual(r.status,403);r.read()
            conn.request('GET','/api/state?key=testing_super_secret')
            r=conn.getresponse();self.assertEqual(r.status,200)
            state=json.loads(r.read());self.assertEqual(state['leader'],'team_02')
            conn.request('POST','/api/state?key=testing_super_secret',body=b'{}')
            r=conn.getresponse();self.assertEqual(r.status,405);r.read()
            conn.request('DELETE','/api/state?key=testing_super_secret')
            r=conn.getresponse();self.assertEqual(r.status,405);r.read()
            conn.request('GET','/?key=testing_super_secret')
            r=conn.getresponse();self.assertEqual(r.status,200)
            html=r.read().decode('utf-8');self.assertIn('DNF',html);self.assertIn('자동 변경',html)
            conn.close()
        finally:srv.shutdown();srv.server_close();t.join(timeout=2)

    def test_session_end_no_stale_ranking(self):
        self.assertEqual(RaceData(self.root).state()['leader'],'team_02')
        (self.root/'race/results/operator_v26_active.json').unlink()
        data=RaceData(self.root).state()
        self.assertEqual(data['state'],'waiting')
        self.assertIsNone(data['leader'])


if __name__=='__main__':unittest.main()
