"""Offline unit/regression tests for the non-invasive v16 controls."""
from pathlib import Path
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest

HERE=Path(__file__).resolve().parents[3]
SCRIPTS=HERE/'overlay/race/scripts'
sys.path.insert(0,str(SCRIPTS))
# Use the exact unchanged v4/v6 inspection modules from the known patch bundle
# when running this self-contained test suite.
# The production installer uses existing v4/v6 scripts unchanged.
# For unit tests, fixture copies make the archive independent of the build machine.
sys.path.insert(0,str(HERE))
import inspection_v4_fixture as _insp
import base_link_check_v6_fixture as _base
sys.modules['inspection_v4']=_insp
sys.modules['base_link_check_v6']=_base
from qualifying_order_v16 import ordered
from vehicle_preflight_v16 import check_entries, summarize_live
spec=importlib.util.spec_from_file_location('v16_installer',HERE/'install_v16.py')
installer=importlib.util.module_from_spec(spec);spec.loader.exec_module(installer)


class V16Test(unittest.TestCase):
    def test_rotate_first_team_and_keep_complete_list(self):
        es=[{'entry_id':f'team_{n:02d}'} for n in (5,2,11,1)]
        self.assertEqual([x['entry_id'] for x in ordered(es,'team_05')],
                         ['team_05','team_11','team_01','team_02'])
    def test_default_and_invalid_team(self):
        es=[{'entry_id':'team_02'},{'entry_id':'team_01'}]
        previous=os.environ.pop('URRC_QUALIFYING_FIRST',None)
        try:self.assertEqual([x['entry_id'] for x in ordered(es)],['team_01','team_02'])
        finally:
            if previous is not None:os.environ['URRC_QUALIFYING_FIRST']=previous
        with self.assertRaisesRegex(ValueError,'등록'):ordered(es,'team_11')
        with self.assertRaisesRegex(ValueError,'형식'):ordered(es,'team_99')
    def test_stage_patch_idempotent(self):
        s="def run():\n    def selected_validate(root):\n        return sorted(all_entries,key=lambda e:e['entry_id'])\n"
        x,changed=installer.patch_stage(s)
        self.assertTrue(changed)
        y,changed2=installer.patch_stage(x)
        self.assertFalse(changed2)
        self.assertEqual(x,y)
    def test_empty_team_source_patch_is_idempotent(self):
        text=(HERE/'team_sync_v7_fixture.py').read_text()
        result,changed=installer.patch_sync(text)
        self.assertTrue(changed)
        self.assertIn('folders=[p for p in folders if any(p.iterdir())]',result)
        again,changed=installer.patch_sync(result)
        self.assertFalse(changed)
        self.assertEqual(result,again)

    def test_actual_sync_skips_only_empty_placeholders(self):
        source=installer.patch_sync((HERE/'team_sync_v7_fixture.py').read_text())[0]
        namespace={'__name__':'v16_sync_test','__file__':str(HERE/'team_sync_v7_fixture.py')}
        exec(compile(source,'team_sync_v7.py','exec'),namespace)
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            for i in range(1,13):
                (root/'src/team'/f'team_{i:02d}').mkdir(parents=True)
            self.assertEqual(namespace['sync'](root,dry_run=True),[])
            (root/'src/team/team_03/README.txt').write_text('incomplete upload')
            with self.assertRaisesRegex(ValueError,'race_entry.json'):
                namespace['sync'](root,dry_run=True)

    def test_stage_patch_rejects_unknown(self):
        with self.assertRaises(ValueError):installer.patch_stage('def run(): pass')
    def create_entrant(self,base,team='team_01',package='team01_race',speed=None,with_base=True):
        root=base/'src/team'/team
        pkg=root/'src'/package
        (pkg/'models').mkdir(parents=True)
        (pkg/'launch').mkdir(parents=True)
        (pkg/'package.xml').write_text('<package><name>'+package+'</name></package>')
        (pkg/'launch/race.launch.py').write_text('# use_sim_time race_start_topic\n')
        (pkg/'models/model.sdf').write_text('''<sdf version="1.9"><model name="car"><static>false</static>
            <link name="%s"><collision name="car"><geometry><box>
            <size>0.4 0.2 0.1</size></box></geometry></collision>
            <sensor name="camera" type="camera"/><sensor name="lidar" type="gpu_lidar"/>
            </link></model></sdf>''' % ('base_link' if with_base else 'other'))
        manifest=dict(entry_id=team,ros_package=package,algorithm_launch_file='race.launch.py',
                      vehicle_model_file='src/'+package+'/models/model.sdf',vehicle_model_format='sdf',
                      speed_topic=speed or '/'+team+'/speed',steering_topic='/'+team+'/steer',
                      lidar_topic='/'+team+'/scan')
        (root/'race_entry.json').write_text(json.dumps(manifest))
        return root
    def test_empty_folders_not_failures(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);(root/'src/team/team_01').mkdir(parents=True)
            report=check_entries(root,'team_01',conversion=False)
            self.assertEqual(report[0]['status'],'EMPTY')
    def test_static_pass_and_no_entrant_mutation(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);p=self.create_entrant(root)
            files={x.relative_to(p):x.read_bytes() for x in p.rglob('*') if x.is_file()}
            report=check_entries(root,'team_01',conversion=False)
            self.assertEqual(report[0]['status'],'STATIC_PASS',report[0])
            self.assertEqual({x.relative_to(p):x.read_bytes() for x in p.rglob('*') if x.is_file()},files)
    def test_official_entry_mismatch_is_explained(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            self.create_entrant(root)
            official=root/'race/entrants/team_01'
            official.mkdir(parents=True)
            data=json.loads((root/'src/team/team_01/race_entry.json').read_text())
            data['speed_topic']='/stale/speed'
            (official/'race_entry.json').write_text(json.dumps(data))
            report=check_entries(root,'team_01',conversion=False)
            self.assertEqual(report[0]['status'],'FAIL')
            self.assertIn('공식 접수본 설정 불일치',' '.join(report[0]['errors']))

    def test_missing_base_and_model_report_fail(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);self.create_entrant(root,with_base=False)
            report=check_entries(root,'team_01',conversion=False)
            self.assertEqual(report[0]['status'],'FAIL')
            self.assertTrue(any('base_link' in x for x in report[0]['errors']))
    def test_duplicate_package_and_topics(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);self.create_entrant(root,'team_01','shared','/duplicate/speed')
            self.create_entrant(root,'team_02','shared','/duplicate/speed')
            result=check_entries(root,conversion=False)
            self.assertEqual(result[0]['status'],'FAIL')
            self.assertEqual(result[1]['status'],'FAIL')
            self.assertIn('중복',' '.join(result[0]['errors']))
    def test_live_no_clock_has_reason(self):
        snapshot={'world_match':True,'teams':{'team_01':{'speed_topic':'/speed','steering_topic':'/steer'}},
                  'samples':{},'ros_topics':{}}
        # install a lightweight v15 name matcher stub in sys.modules for testing
        import types
        matcher=types.SimpleNamespace(topic_is_team=lambda *a:False)
        old=sys.modules.get('environment_diagnostic_v15')
        sys.modules['environment_diagnostic_v15']=matcher
        try:result=summarize_live(snapshot,'team_01')
        finally:
            if old is None:sys.modules.pop('environment_diagnostic_v15',None)
            else:sys.modules['environment_diagnostic_v15']=old
        self.assertEqual(result['status'],'FAIL')
        self.assertTrue(any(r['area']=='clock' for r in result['findings']))
    def test_shell_qualifying_first_team_and_args(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            (root/'race_v16.sh').write_bytes((HERE/'overlay/race_v16.sh').read_bytes())
            (root/'race_v13.sh').write_text(
                '#!/usr/bin/env bash\nprintf "stage=%s\\nfirst=%s\\nrest=%s\\n" "$1" "${URRC_QUALIFYING_FIRST-}" "$2"\n')
            result=subprocess.run(['bash',str(root/'race_v16.sh'),'qualifying','team_02','--track','monza'],
                                  capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('stage=qualifying',result.stdout)
            self.assertIn('first=team_02',result.stdout)
            self.assertIn('rest=--track',result.stdout)
            rejected=subprocess.run(['bash',str(root/'race_v16.sh'),'qualifying','team_99'],
                                    capture_output=True,text=True)
            self.assertNotEqual(rejected.returncode,0)
            self.assertIn('잘못된 참가팀',rejected.stderr)

    def test_installer_only_changes_operator_files(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            for item in ['race/scripts','race/tests','src/team/team_01','race/entrants/team_01','race/results/runs/old']:
                (root/item).mkdir(parents=True)
            for item in ['inspection_v4.py','base_link_check_v6.py','environment_diagnostic_v15.py']:
                (root/'race/scripts'/item).write_text('# original\n')
            (root/'race_v13.sh').write_text('#!/bin/bash\n')
            (root/'race/scripts/team_sync_v7.py').write_text((HERE/'team_sync_v7_fixture.py').read_text())
            (root/'race/scripts/stage_v2.py').write_text(
                "def run():\n    def selected_validate(root):\n        return sorted(all_entries,key=lambda e:e['entry_id'])\n")
            team=root/'src/team/team_01/car.txt';team.write_text('must preserve')
            audit=root/'race/results/runs/old/record.json';audit.write_text('{}')
            entr=root/'race/entrants/team_01/data.txt';entr.write_text('protected')
            installer.install(root,HERE)
            installer.install(root,HERE)
            self.assertEqual(team.read_text(),'must preserve')
            self.assertEqual(entr.read_text(),'protected')
            self.assertEqual(audit.read_text(),'{}')
            self.assertTrue((root/'race_v16.sh').is_file())
            self.assertEqual((root/'race/scripts/stage_v2.py').read_text().count(installer.MARKER),1)
            self.assertEqual((root/'race/scripts/team_sync_v7.py').read_text().count(installer.SYNC_MARKER),1)


if __name__=='__main__':unittest.main()
