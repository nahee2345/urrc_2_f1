import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import bracket_v12 as bracket


def rows(ids, finished=None):
    return [{'entry_id':eid,'rank':i,'status':'finished' if finished is None or eid in finished else 'dnf'}
            for i,eid in enumerate(ids,1)]


def write(base,name,items,valid=True):
    (base/(name+'.json')).write_text(json.dumps({'valid':valid,'state':'complete','provisional':False,'results':items}),encoding='utf-8')


class TestBracketV12(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.folder=Path(self.tmp.name)
    def qualifier(self,n):
        ids=[f'team_{i:02d}' for i in range(1,n+1)]
        write(self.folder,'qualifying',rows(ids))
        return ids
    def test_one_team_fails_semifinal(self):
        self.qualifier(1)
        with self.assertRaisesRegex(ValueError,'2~12'):bracket.eligible_ids('semi_a',self.folder)
    def test_two_team_full(self):
        ids=self.qualifier(2)
        self.assertEqual(bracket.eligible_ids('semi_a',self.folder),ids)
        self.assertEqual(bracket.eligible_ids('semi_b',self.folder),[])
        write(self.folder,'semi_a',rows(ids))
        self.assertEqual(bracket.eligible_ids('final',self.folder),ids)
    def test_three_team_full(self):
        ids=self.qualifier(3)
        self.assertEqual(bracket.eligible_ids('semi_a',self.folder),ids)
        self.assertEqual(bracket.eligible_ids('semi_b',self.folder),[])
        write(self.folder,'semi_a',rows(ids))
        write(self.folder,'semi_b',rows(['fake']))  # stale B never used
        self.assertEqual(bracket.eligible_ids('final',self.folder),ids)
    def test_four_team_regular_split(self):
        self.qualifier(4)
        self.assertEqual(bracket.eligible_ids('semi_a',self.folder),['team_01','team_03'])
        self.assertEqual(bracket.eligible_ids('semi_b',self.folder),['team_02','team_04'])
    def test_six_team_final_six(self):
        self.qualifier(6)
        for leg in ('semi_a','semi_b'):
            write(self.folder,leg,rows(bracket.eligible_ids(leg,self.folder)))
        self.assertEqual(bracket.eligible_ids('final',self.folder),[
            'team_01','team_02','team_03','team_04','team_05','team_06'])
    def test_seven_team_final_seven(self):
        self.qualifier(7)
        for leg in ('semi_a','semi_b'):
            write(self.folder,leg,rows(bracket.eligible_ids(leg,self.folder)))
        self.assertEqual(bracket.eligible_ids('final',self.folder),[
            'team_01','team_02','team_03','team_04','team_05','team_06','team_07'])
    def test_eight_team_wildcard_by_qualifying_rank(self):
        self.qualifier(8)
        for leg in ('semi_a','semi_b'):
            write(self.folder,leg,rows(bracket.eligible_ids(leg,self.folder)))
        finalists=bracket.eligible_ids('final',self.folder)
        self.assertEqual(finalists,[f'team_{i:02d}' for i in range(1,8)])
        self.assertNotIn('team_08',finalists)
    def test_twelve_team_limits(self):
        self.qualifier(12)
        for leg in ('semi_a','semi_b'):
            ids=bracket.eligible_ids(leg,self.folder)
            self.assertEqual(len(ids),6)
            write(self.folder,leg,rows(ids))
        finals=bracket.eligible_ids('final',self.folder)
        self.assertEqual(len(finals),7)
        self.assertEqual(finals[-1],'team_07')
    def test_missing_semi_results_fail(self):
        self.qualifier(4)
        with self.assertRaises(ValueError):bracket.eligible_ids('final',self.folder)
    def test_invalid_previous(self):
        write(self.folder,'qualifying',rows(['team_01','team_02']),valid=False)
        with self.assertRaises(ValueError):bracket.eligible_ids('semi_a',self.folder)
    def test_final_min_two_finishers(self):
        ids=self.qualifier(3)
        write(self.folder,'semi_a',rows(ids,finished={ids[0]}))
        with self.assertRaisesRegex(ValueError,'최소 2팀'):bracket.eligible_ids('final',self.folder)
    def test_semifinal_member_mismatch(self):
        self.qualifier(4)
        write(self.folder,'semi_a',rows(['team_01','team_10']))
        write(self.folder,'semi_b',rows(['team_02','team_04']))
        with self.assertRaises(ValueError):bracket.eligible_ids('final',self.folder)
    def test_final_prefers_actual_finished(self):
        ids=self.qualifier(4)
        write(self.folder,'semi_a',rows([ids[0],ids[2]],finished={ids[0]}))
        write(self.folder,'semi_b',rows([ids[1],ids[3]],finished={ids[1],ids[3]}))
        self.assertEqual(bracket.eligible_ids('final',self.folder),[ids[0],ids[1],ids[3]])

if __name__=='__main__':unittest.main()
