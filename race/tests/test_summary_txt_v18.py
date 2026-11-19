"""ROS/Gazebo-free tests: v18 text summary and installed wrapper."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPT=Path(__file__).resolve().parents[1]/'scripts'/'summary_txt_v18.py'
spec=importlib.util.spec_from_file_location('summary_txt_v18',SCRIPT)
v18=importlib.util.module_from_spec(spec);spec.loader.exec_module(v18)


def repo(tmp):
    root=Path(tmp)
    (root/'race/scripts').mkdir(parents=True)
    (root/'race/scripts/race_manager.py').write_text('# race stub\n')
    (root/'race/results').mkdir(parents=True)
    (root/'src/team/team_01').mkdir(parents=True)
    (root/'src/team/team_01/race_entry.json').write_text('{}')
    (root/'src/team/team_02').mkdir(parents=True)
    (root/'src/team/team_02/race_entry.json').write_text('{}')
    return root


class SummaryTests(unittest.TestCase):
    def test_empty_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=repo(tmp)
            output=v18.generate(root)
            self.assertEqual(len(list(output.glob('*.txt'))),8) # summary, 5 stages, 2 teams
            self.assertIn('공식 경기: 미실행', (output/'qualifying.txt').read_text())
            self.assertIn('차량검수 | 미검수', (output/'team_01.txt').read_text())

    def test_official_finished_dnf_and_track(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=repo(tmp)
            src=root/'race/results/qualifying.json'
            src.write_text(json.dumps({'valid':True,'state':'complete','track':'monza',
                'results':[{'entry_id':'team_01','rank':1,'status':'finished','laps':2,'time_seconds':67.35},
                    {'entry_id':'team_02','rank':2,'status':'DNF','laps':1,'reason':'출발 후 정지','time_seconds':None}]}))
            original=src.read_bytes()
            output=v18.generate(root)
            body=(output/'qualifying.txt').read_text()
            self.assertIn('monza | COMPLETE',body)
            self.assertIn('team_01 | 1위 | PASS | 2/2랩 | 01:07.350',body)
            self.assertIn('team_02 | 2위 | DNF | 1/2랩',body)
            self.assertIn('출발 후 정지',body)
            self.assertIn('예선 | 1위 | PASS', (output/'team_01.txt').read_text())
            self.assertEqual(src.read_bytes(),original)

    def test_invalid_race_is_not_misrepresented_as_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=repo(tmp)
            (root/'race/results/final.json').write_text(json.dumps({'valid':False,
                'state':'invalid','track':'monaco','error':'시계 중단',
                'results':[{'entry_id':'team_01','status':'finished','laps':5,'time_seconds':59}]}))
            body=(v18.generate(root)/'final.txt').read_text()
            self.assertIn('INVALID',body)
            self.assertIn('INVALID(개별 완주 무효)',body)
            self.assertIn('경기 오류 | 시계 중단',body)

    def test_inspection_reports_and_reason(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=repo(tmp)
            cache=root/'race/results/inspections';cache.mkdir()
            (cache/'current_all.json').write_text(json.dumps({'registered':2,'teams':[
                {'team':'team_01','status':'FAIL','errors':['base_link 중심 오류','차량 초과'],'warnings':[]},
                {'team':'team_02','status':'REVIEW','errors':[],'warnings':['카메라 확인 필요']}]}))
            body=(v18.generate(root)/'vehicles.txt').read_text()
            self.assertIn('team_01 | FAIL | base_link 중심 오류 (+1)',body)
            self.assertIn('team_02 | REVIEW | 카메라 확인 필요',body)

    def test_rehearsal_is_not_shown_as_official(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=repo(tmp)
            source=root/'race/previews/v17_qualifying_xyz/report.json'
            source.parent.mkdir(parents=True)
            source.write_text(json.dumps({'mode':'REHEARSAL_NOT_OFFICIAL','stage':'qualifying',
                'track':'monza','status':'OBSERVATION_COMPLETE',
                'details':{'team_01':{'status':'PASS','laps':2}}}))
            body=(v18.generate(root)/'qualifying.txt').read_text()
            self.assertIn('공식 경기: 미실행',body)
            self.assertIn('최신 모의주행 (비공식)',body)
            self.assertIn('team_01 | PASS | 2/2랩',body)
            self.assertIn('연습 | PASS', (root/'race/results/summary_txt/team_01.txt').read_text())

    def test_updates_are_idempotent_and_do_not_delete_existing_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=repo(tmp)
            output=v18.generate(root)
            custom=output/'notes.txt';custom.write_text('my content')
            before={p.name:p.read_bytes() for p in output.glob('*.txt')}
            v18.generate(root)
            self.assertEqual(custom.read_text(),'my content')
            self.assertEqual({p.name:p.read_bytes() for p in output.glob('*.txt')},before|{'notes.txt':b'my content'})

    def test_malformed_result_does_not_break_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=repo(tmp)
            (root/'race/results/semi_a.json').write_text('{invalid}')
            self.assertIn('공식 경기: 미실행', (v18.generate(root)/'semi_a.txt').read_text())

    def test_speed_time_formatting(self):
        self.assertEqual(v18.duration(367.9),'06:07.900')
        self.assertEqual(v18.duration(None),'-')
        self.assertEqual(v18.duration(float('inf')),'-')


class InstallerTests(unittest.TestCase):
    def test_install_repeated_no_team_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'repo';pkg=Path(tmp)/'pkg'
            (root/'race/scripts').mkdir(parents=True)
            (root/'src/team/team_01').mkdir(parents=True)
            participant=root/'src/team/team_01/vehicle.sdf';participant.write_text('keep')
            for name in ('race_v17.sh','race/scripts/race_manager.py','race/scripts/vehicle_preflight_v16.py'):
                p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('keep')
            # Test with current package folder instead of creating fake content.
            installer=Path(__file__).resolve().parents[3]/'install_v18.py'
            s=importlib.util.spec_from_file_location('install_v18',installer)
            m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
            package=installer.parent
            m.install(root,package)
            first=(root/'race/scripts/summary_txt_v18.py').read_bytes()
            m.install(root,package)
            self.assertEqual(participant.read_text(),'keep')
            self.assertEqual((root/'race/scripts/summary_txt_v18.py').read_bytes(),first)
            self.assertFalse((root/'race/results').exists())


if __name__=='__main__':unittest.main()

class ShellWrapperTests(unittest.TestCase):
    def _root(self, tmp):
        root=repo(tmp)
        package=Path(__file__).resolve().parents[3]
        (root/'race_v18.sh').write_bytes((package/'overlay/race_v18.sh').read_bytes())
        (root/'race/scripts/summary_txt_v18.py').write_bytes((package/'overlay/race/scripts/summary_txt_v18.py').read_bytes())
        return root

    def test_stage_nonzero_updates_text_preserves_exit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=self._root(tmp)
            code='''#!/usr/bin/env bash
printf '%s\n' '{"valid":false,"state":"invalid","track":"monza","error":"test interruption","results":[]}' > "$(dirname "$0")/race/results/qualifying.json"
exit 7
'''
            (root/'race_v17.sh').write_text(code)
            result=subprocess.run(['bash',str(root/'race_v18.sh'),'qualifying','team_01'],capture_output=True,text=True)
            self.assertEqual(result.returncode,7, result.stderr)
            self.assertIn('test interruption',(root/'race/results/summary_txt/qualifying.txt').read_text())

    def test_vehicle_failure_writes_team_reason(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=self._root(tmp)
            code='''#!/usr/bin/env bash
[[ "$1" == vehicles ]] || exit 8
report=''
while (( $# )); do
  if [[ "$1" == --report ]]; then report="$2"; break; fi
  shift
done
[[ -n "$report" ]] || exit 9
printf '%s\n' '{"registered":1,"teams":[{"team":"team_01","status":"FAIL","errors":["camera missing"],"warnings":[]}]}' > "$report"
exit 2
'''
            (root/'race_v17.sh').write_text(code)
            result=subprocess.run(['bash',str(root/'race_v18.sh'),'vehicles','--team','team_01'],capture_output=True,text=True)
            self.assertEqual(result.returncode,2,result.stderr)
            self.assertIn('team_01 | FAIL | camera missing',(root/'race/results/summary_txt/vehicles.txt').read_text())
