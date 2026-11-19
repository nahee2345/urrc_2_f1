import unittest,sys, tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from race_monitor_v6 import summarize,follow_jsonl

class MonitorTest(unittest.TestCase):
    def test_first_gate_time(self):
        events=[{'type':'arm','entry_id':'team_01','start_time':10},
                {'type':'crossing','entry_id':'team_01','gate':0,'time':12},
                {'type':'pose','entry_id':'team_01','sample':[15,0,0,0]},
                {'type':'crossing','entry_id':'team_01','gate':1,'time':13}]
        row=summarize(events,['team_01'],100)[0]
        self.assertEqual(row['elapsed_s'],3)
        self.assertEqual(row['laps'],0)
    def test_rank(self):
        events=[]
        for x in ['team_01','team_02']:events.append({'type':'crossing','entry_id':x,'gate':0,'time':10})
        events.append({'type':'crossing','entry_id':'team_02','gate':1,'time':11})
        rows=summarize(events,['team_01','team_02'],100)
        self.assertEqual(rows[0]['team'],'team_02')
        self.assertEqual(rows[1]['gap_checkpoints'],1)
    def test_incremental(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'test.jsonl';p.write_bytes(b'{"type":"arm"}\n{"type":"pose"')
            a,n,buf=follow_jsonl(p,0,b'');self.assertEqual(len(a),1)
            with p.open('ab') as f:f.write(b'}\n')
            b,n,buf=follow_jsonl(p,n,buf);self.assertEqual(len(b),1)
if __name__=='__main__':unittest.main()
