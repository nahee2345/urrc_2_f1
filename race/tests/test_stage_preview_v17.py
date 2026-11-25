"""ROS-free preflight tests for URRC v17 operator-only preview and rehearsal."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

SCRIPT=Path(__file__).resolve().parents[1]/'scripts'/'stage_preview_v17.py'
spec=importlib.util.spec_from_file_location('stage_preview_v17',SCRIPT)
v17=importlib.util.module_from_spec(spec);sys.modules[spec.name]=v17;spec.loader.exec_module(v17)


class ChoiceTests(unittest.TestCase):
    def test_user_track_override_does_not_write_assignment(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            track,kind=v17.choose_track(root,'qualifying','monza')
            self.assertEqual(track,'monza')
            self.assertIn('명시적',kind)
            self.assertFalse((root/'race').exists())

    def test_stage_map_comes_from_existing_assignment(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            target=root/'race/results/v3_maps.json';target.parent.mkdir(parents=True)
            target.write_text(json.dumps({'maps':{
                'qualifying':'spa','semi_a':'cota','semi_b':'baku','final':'monaco'}}))
            before=target.read_bytes()
            self.assertEqual(v17.choose_track(root,'semi_b',None)[0],'baku')
            self.assertEqual(before,target.read_bytes())

    def test_invalid_assignment_refuses_to_repair(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            target=root/'race/results/v3_maps.json';target.parent.mkdir(parents=True)
            target.write_text(json.dumps({'maps':{
                'qualifying':'spa','semi_a':'spa','semi_b':'baku','final':'monaco'}}))
            with self.assertRaisesRegex(ValueError,'중복'):
                v17.choose_track(root,'semi_a',None)

    def test_fallback_does_not_create_event_state(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            self.assertEqual(v17.choose_track(root,'final',None)[0],'monaco')
            self.assertFalse((root/'race').exists())

    def test_invalid_team_and_duplicate_rejected(self):
        with self.assertRaisesRegex(ValueError,'ID'):
            v17.validate_team_names(['team_13'])
        with self.assertRaisesRegex(ValueError,'중복'):
            v17.validate_team_names(['team_01','team_01'])
        with self.assertRaisesRegex(ValueError,'--team'):
            v17.validate_team_names([])

    def test_rehearsal_stage_limits(self):
        entries=[{'entry_id':f'team_{i:02d}'} for i in range(1,13)]
        self.assertEqual(len(v17.selected_entries(entries,[f'team_{i:02d}' for i in range(1,8)],'final')),7)
        with self.assertRaisesRegex(ValueError,'최대 6대'):
            v17.selected_entries(entries,[f'team_{i:02d}' for i in range(1,8)],'semi_a')
        with self.assertRaisesRegex(ValueError,'최대 1대'):
            v17.selected_entries(entries,['team_01','team_02'],'qualifying')
        with self.assertRaisesRegex(ValueError,'공식 접수본'):
            v17.selected_entries(entries[:2],['team_01','team_02','team_03'],'final')

    def test_report_pass_only_when_target_laps_complete(self):
        self.assertEqual(v17.report_status({'laps':3,'required_laps':3,'finish_time':20.}),'PASS')
        self.assertEqual(v17.report_status({'laps':2,'required_laps':3,'finish_time':None}),'INCOMPLETE')
        self.assertEqual(v17.report_status({'laps':3,'required_laps':3,'finish_time':None}),'INCOMPLETE')


class PreviewTests(unittest.TestCase):
    def test_preview_starts_world_but_not_official_results(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            world=root/'src/urrc_track_gazebo/worlds/monza_race.sdf'
            gui=world.with_name('monza_grid.gui.config')
            world.parent.mkdir(parents=True)
            (root/'src/urrc_track_gazebo/models').mkdir()
            world.write_text('<sdf/>');gui.write_text('<gui/>')
            marker=root/'race/results/qualifying.json';marker.parent.mkdir(parents=True)
            marker.write_text('{"official":true}')
            class P:
                pid=111
                def wait(self):return 0
                def poll(self):return 0
            with patch.object(v17.subprocess,'Popen',return_value=P()) as spawn,\
                 patch.object(v17,'stop_process_group') as stop:
                self.assertEqual(v17.preview_map(root,'qualifying','monza'),0)
                command=spawn.call_args.args[0]
                self.assertTrue(command[-1].endswith('monza_race.sdf'))
                self.assertIn('GZ_PARTITION',spawn.call_args.kwargs['env'])
                self.assertEqual(marker.read_text(),'{"official":true}')
                self.assertFalse((root/'race/previews').exists())
                self.assertTrue(stop.called)


class RehearsalTests(unittest.TestCase):
    def test_fake_race_scoring_isolation(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            (root/'race/install').mkdir(parents=True)
            official=root/'race/results/final.json';official.parent.mkdir(parents=True)
            official.write_text('{"official":"unchanged"}')
            diag=root/'race/scripts/environment_diagnostic_v15.py'
            diag.parent.mkdir(parents=True,exist_ok=True);diag.write_text('# stub')
            entries=[{'entry_id':x,'ros_package':'dummy_race','_model':'/tmp/fake'} for x in ('team_01','team_02')]
            calls=[]
            class P:
                def poll(self):return None
            class M:
                def wait_seen(self,*_):return True
                def health(self,*_):pass
                def snapshot(self,eid):return dict(laps=5,finish_time=3.,
                    max_displacement_m=10.,speed_messages_after_release=15,
                    max_abs_speed_command=4.)
                def diagnostic_line(self,eid):return 'fake telemetry'
                def close(self):calls.append('monitor.close')
            class Gate:
                pubs={eid:types.SimpleNamespace(get_subscription_count=lambda:1)
                      for eid in ('team_01','team_02')}
                def ready(self,*a,**k):return []
                def publish(self,ids,value):calls.append(('gate',value))
                def close(self):calls.append('gate.close')
            def fake_spawn(root,track,e,slot,env,run_dir):
                generated=run_dir/'runtime_models'/f"{e['entry_id']}.sdf"
                generated.parent.mkdir(parents=True,exist_ok=True)
                generated.write_text('temporary file')
                e['_prepared_model']=str(generated)
            fake=types.SimpleNamespace(
                load_entries=lambda root:entries,
                grid=lambda *x:[{}]*12,
                read_json=lambda *x:{'lights_out_min_delay_seconds':.2},
                start_world=lambda *x:(P(),P(),{'GZ_PARTITION':'fake'},types.SimpleNamespace(close=lambda:None)),
                PoseLapMonitor=lambda *x:M(), StartGate=lambda *x:Gate(),
                spawn=fake_spawn, launch_algo=lambda *x:P(),
                start_sequence=lambda *x:1.0,
                stop=lambda p:calls.append('stop'),
                remove=lambda *x:calls.append('remove'))
            def sensor_popen(cmd,**kwargs):
                path=Path(cmd[cmd.index('--output')+1])
                path.write_text(json.dumps({'world_match':True,'samples':{'/clock':{'count':12}},
                       'findings':{eid:[{'area':'camera','status':'PASS'}] for eid in ('team_01','team_02')}}))
                return types.SimpleNamespace(poll=lambda:0)
            with patch.dict(sys.modules,{'race_manager':fake, 'rclpy': types.SimpleNamespace(ok=lambda:False)}),\
                 patch.object(v17.subprocess,'Popen',side_effect=sensor_popen),\
                 patch.object(v17,'preflight_selected',return_value=None),\
                 patch.object(v17.subprocess,'run',return_value=types.SimpleNamespace(returncode=0)),\
                 patch.dict(os_environ(),{'GZ_PARTITION':'before'}):
                self.assertEqual(v17.run_rehearsal(root,'final','monaco',
                    ['team_01','team_02'],10),0)
            self.assertEqual(official.read_text(),'{"official":"unchanged"}')
            reports=list((root/'race/previews').glob('v17_*/report.json'))
            self.assertEqual(len(reports),1)
            report=json.loads(reports[0].read_text())
            self.assertEqual(report['status'],'OBSERVATION_COMPLETE')
            self.assertEqual(report['details']['team_01']['status'],'PASS')
            self.assertEqual(report['sensor_status'],'CAPTURED')
            self.assertTrue(report['sensor_world_match'])
            self.assertTrue(Path(report['sensor_report']).is_file())
            self.assertFalse(list((reports[0].parent/'runtime_models').glob('*.sdf')))
            self.assertIn('remove',calls)
            self.assertIn('stop',calls)



class InstallerTests(unittest.TestCase):
    def test_operator_overlay_preserves_team_and_official_results(self):
        package=SCRIPT.parents[3]  # urrc_f1_v17/overlay/race/scripts/ -> ZIP root
        installer=package/'install_v17.py'
        spec_install=importlib.util.spec_from_file_location('install_v17',installer)
        module=importlib.util.module_from_spec(spec_install)
        spec_install.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            for rel in ('race/scripts/race_manager.py',
                        'race/scripts/vehicle_preflight_v16.py','race_v13.sh','race_v16.sh'):
                path=root/rel;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('original')
            (root/'src/team/team_01').mkdir(parents=True)
            team_file=root/'src/team/team_01/secret.stl';team_file.write_bytes(b'important participant data')
            result=root/'race/results/qualifying.json';result.parent.mkdir(parents=True)
            result.write_bytes(b'preserve official record')
            module.install(root,package)
            first=(root/'race/scripts/stage_preview_v17.py').read_bytes()
            module.install(root,package)  # idempotent
            self.assertEqual(first,(root/'race/scripts/stage_preview_v17.py').read_bytes())
            self.assertEqual(team_file.read_bytes(),b'important participant data')
            self.assertEqual(result.read_bytes(),b'preserve official record')
            self.assertFalse((root/'race/previews').exists())

    def test_installer_rejects_incomplete_workspace(self):
        package=SCRIPT.parents[3]
        installer=package/'install_v17.py'
        spec_install=importlib.util.spec_from_file_location('install_v17_missing',installer)
        module=importlib.util.module_from_spec(spec_install)
        spec_install.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            with self.assertRaisesRegex(ValueError,'필수 기존 파일'):
                module.install(root,package)
            self.assertFalse((root/'race_v17.sh').exists())

def os_environ():
    import os
    return os.environ


if __name__=='__main__':unittest.main()
