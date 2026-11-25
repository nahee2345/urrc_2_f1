import json,os
from pathlib import Path
import pytest
from team_01_race.practice import map_spec,state_dir,role_lock,process_start,alive,load_session


def test_external_filename_and_world_use_explicit_pose_without_operator_code(tmp_path):
 p=tmp_path/'a different map.sdf';p.write_text('<sdf version="1.9"><world name="arbitrary_course"/></sdf>')
 spec=map_spec(tmp_path/'team_01','monza',p,spawn=[1.,2.,3.,.4])
 assert spec['world']=='arbitrary_course' and spec['slot']==dict(x=1.,y=2.,road_z=3.,yaw=.4)
 assert not spec['has_lights']


def test_common_grid_uses_first_position_and_real_elevation(tmp_path,monkeypatch):
 package=tmp_path/'common';(package/'worlds').mkdir(parents=True);(package/'models').mkdir();(package/'tracks/processed').mkdir(parents=True)
 (package/'worlds/monza_race.sdf').write_text('<sdf><world name="monza"><model name="race_start_gantry"/></world></sdf>')
 (package/'tracks/processed/monza.json').write_text(json.dumps(dict(grid=[dict(position=2,x=8,y=9,road_z=7,yaw=1),dict(position=1,x=3,y=4,road_z=2,yaw=.5)])))
 monkeypatch.setenv('URRC_MAP_PACKAGE',str(package))
 spec=map_spec(tmp_path/'team_01','monza')
 assert spec['slot']['x']==3 and spec['slot']['road_z']==2 and spec['has_lights']


def test_missing_or_nonfinite_spawn_is_rejected(tmp_path):
 p=tmp_path/'empty.sdf';p.write_text('<sdf><world name="external"/></sdf>')
 with pytest.raises(RuntimeError,match='출발 위치'):map_spec(tmp_path,'monza',p)
 with pytest.raises(ValueError,match='유효하지 않은 출발 위치'):map_spec(tmp_path,'monza',p,spawn=[0.,0.,float('nan'),0.])


def test_relocated_folder_has_separate_session_and_duplicate_role_is_rejected(tmp_path,monkeypatch):
 monkeypatch.delenv('URRC_PRACTICE_STATE_DIR',raising=False)
 assert state_dir(tmp_path/'old/team_01')!=state_dir(tmp_path/'new/team_01')
 with role_lock(tmp_path/'session',2):
  with pytest.raises(RuntimeError,match='이미 실행 중'):
   with role_lock(tmp_path/'session',2):pass


def test_session_cannot_reuse_a_recycled_pid_or_closing_world(tmp_path):
 pid=os.getpid();start=process_start(pid)
 assert alive(pid,start) and not alive(pid,'not-the-start-time')
 (tmp_path/'session.json').write_text(json.dumps(dict(owner_pid=pid,owner_start='stale',closing=False)))
 with pytest.raises(RuntimeError,match='종료'):load_session(tmp_path)
 (tmp_path/'session.json').write_text(json.dumps(dict(owner_pid=pid,owner_start=start,closing=True)))
 with pytest.raises(RuntimeError,match='종료'):load_session(tmp_path)


def test_debug_topics_use_the_practice_domain_and_preserve_extra_flags(tmp_path,monkeypatch):
 from types import SimpleNamespace
 from team_01_race.practice import watch_role
 connection=dict(ROS_DOMAIN_ID='71',GZ_PARTITION='practice_test',URRC_PRACTICE_SESSION='token')
 (tmp_path/'session.json').write_text(json.dumps(dict(owner_pid=os.getpid(),owner_start=process_start(os.getpid()),closing=False,connection=connection)))
 calls=[];monkeypatch.setattr(os,'execvpe',lambda *a:calls.append(a))
 watch_role(7,SimpleNamespace(launch_args=['--once']),tmp_path)
 executable,args,env=calls[0]
 assert executable=='ros2' and '/team_01/debug/speed_control' in args and args[-1]=='--once'
 assert env['ROS_DOMAIN_ID']=='71' and env['GZ_PARTITION']=='practice_test'


def test_debug_topics_keep_operator_environment_without_practice(tmp_path,monkeypatch):
 from types import SimpleNamespace
 from team_01_race.practice import watch_role
 monkeypatch.setenv('ROS_DOMAIN_ID','12');calls=[];monkeypatch.setattr(os,'execvpe',lambda *a:calls.append(a))
 watch_role(6,SimpleNamespace(launch_args=[]),tmp_path)
 assert calls[0][2]['ROS_DOMAIN_ID']=='12'
