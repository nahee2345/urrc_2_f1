import sys, pathlib, unittest, importlib.util
HERE=pathlib.Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(HERE))
import team_sync_v7, follow_v5, admin_control
class V9Limits(unittest.TestCase):
    def test_teams(self):
        self.assertTrue(team_sync_v7.TEAM.fullmatch('team_12'))
        self.assertTrue(team_sync_v7.TEAM.fullmatch('team_01'))
        self.assertFalse(team_sync_v7.TEAM.fullmatch('team_13'))
    def test_camera(self):
        self.assertEqual(follow_v5.KEY_GRID['['],11)
        self.assertEqual(follow_v5.KEY_GRID[']'],12)
    def test_admin(self):
        self.assertTrue(admin_control.ENTRY_RE.fullmatch('team_12'))
        self.assertFalse(admin_control.ENTRY_RE.fullmatch('team_13'))

class V9StageSelection(unittest.TestCase):
    def test_twelve_team_brackets(self):
        import stage_v2, tempfile, json
        with tempfile.TemporaryDirectory() as d:
            p=pathlib.Path(d)
            qualifying=[{'rank':i,'entry_id':f'team_{i:02d}','status':'finished'} for i in range(1,13)]
            (p/'qualifying.json').write_text(json.dumps({'valid':True,'state':'complete','provisional':False,'results':qualifying}))
            self.assertEqual(stage_v2.eligible_ids('semi_a',p),[f'team_{i:02d}' for i in range(1,13,2)])
            self.assertEqual(stage_v2.eligible_ids('semi_b',p),[f'team_{i:02d}' for i in range(2,13,2)])
            for stage,subset in [('semi_a',stage_v2.eligible_ids('semi_a',p)),('semi_b',stage_v2.eligible_ids('semi_b',p))]:
                rows=[{'rank':i,'entry_id':eid,'status':'finished'} for i,eid in enumerate(subset,1)]
                (p/f'{stage}.json').write_text(json.dumps({'valid':True,'state':'complete','provisional':False,'results':rows}))
            finalists=stage_v2.eligible_ids('final',p)
            self.assertEqual(len(finalists),6)
            self.assertEqual(len(set(finalists)),6)
