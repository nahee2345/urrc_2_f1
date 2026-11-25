"""Safety boundaries of committed trajectory memory, including live obstacles."""
import math,json,sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.navigation import DecisionEngine,Obstacle,valid_path
from team_01_race.corner import CornerDetector
from team_01_race.corner_recovery import CornerRecovery,RecoveryReceiver,wire_context,transformed_paths
from team_01_race.lidar_filter import CameraWallFilter


def geometry(sign=1.):
 x=np.arange(.1,5.01,.1);p=np.column_stack((x,sign*.175*x*x));return p,p+[0.,1.2],p-[0.,1.2]


def committed():
 e=DecisionEngine(vehicle_width=.24,wheelbase=.34,cruise=7.,require_official=False)
 e.set_light('LIGHTS_OUT');e.controller_ready=True;e.actual_speed=1.2
 c=CornerDetector();r=CornerRecovery()
 for i in range(5):
  t=1+i*.03;p,l,right=geometry();e.camera(p,l,right,1.,(True,True),t,t,stamp=t)
  e.lidar([],t,stamp=t);r.health(True,t);r.note_input(t,t,True,1.,(True,True),'')
  c.update(e.path,t,t,speed=1.2);out=c.constrain(e.step(t),t,1.2);r.observe_safe(out,c,e,t,t)
 assert r.committed and r.memory is not None and c.state=='CORNER_COMMIT'
 return e,c,r,1.12


def lost(e,r,t,forward=False):
 e.camera(None,None,None,0.,(False,False),t,t,stamp=t)
 r.health(True,t);r.note_input(t,t,False,0.,(False,False),'path_invalid',forward)
 e.lidar([],t,stamp=t)
 return r.prepare(t,t,1.2,.08)


@pytest.mark.parametrize('frames',[1,3,5])
def test_short_loss_holds_live_forward_trajectory_without_stop(frames):
 e,c,r,t=committed()
 for i in range(1,frames+1):
  ctx=lost(e,r,t+.03*i);out=e.recovery_step(t+.03*i,ctx)
  assert out['enabled'] and out['speed']==1.2 and valid_path(out['path'])
  assert np.all(out['path'][:,0]>0) and r.direction=='LEFT'
 assert r.loss_frames==frames and r.memory['stamp']==t


def test_hold_timeout_uses_original_good_frame_and_images_stay_alive():
 e,c,r,t=committed()
 for i in range(1,12):assert lost(e,r,t+i*.03) is not None
 assert lost(e,r,t+.51) is None
 assert r.stop_reason=='path_lost_timeout' and r.expired


def test_image_timeout_is_not_path_invalid():
 e,c,r,t=committed();ctx=lost(e,r,t+.1);assert ctx
 r.health(False,t+.11)
 assert r.prepare(t+.12,t+.12,1.2,.08) is None and r.stop_reason=='camera_timeout'


@pytest.mark.parametrize('kind,expected',[('ttc','critical_ttc'),('obstacle','confirmed_collision_risk')])
def test_live_obstacle_during_hold_stops_without_waiting_for_deadline(kind,expected):
 e,c,r,t=committed();ctx=lost(e,r,t+.05);x=.7 if kind=='ttc' else 2.
 y=float(np.interp(x,ctx['paths']['selected'][:,0],ctx['paths']['selected'][:,1]))
 e.lidar([Obstacle('car',x,y,.3,.3,1.)],t+.05,stamp=t+.05)
 out=e.recovery_step(t+.05,ctx)
 assert not out['enabled'] and out['speed']==0. and out['reason']==expected


def test_lidar_disconnection_still_stops_during_hold():
 e,c,r,t=committed();ctx=lost(e,r,t+.3);e.lidar_time=t
 out=e.recovery_step(t+.3,ctx)
 assert not out['enabled'] and out['reason']=='lidar_timeout'


def test_fresh_empty_lidar_scan_does_not_invent_spatial_fusion_skew_during_hold():
 e,c,r,t=committed();ctx=lost(e,r,t+.22)
 e.lidar([],t+.04,stamp=t+.04)  # Fresh at 0.18 s; no footprint to fuse.
 assert e.recovery_step(t+.22,ctx)['enabled']
 e.lidar([Obstacle('car',3.,0.,.3,.3,1.)],t+.04,stamp=t+.04)
 assert e.recovery_step(t+.22,ctx)['reason']=='sensor_timestamp_skew'
 e.lidar([],t-.10,stamp=t-.10)
 assert e.recovery_step(t+.22,ctx)['reason']=='lidar_timeout'


def test_external_red_official_and_controller_stop_cannot_be_bypassed():
 for barrier in ('external','red','official','ready'):
  e,c,r,t=committed();ctx=lost(e,r,t+.05)
  if barrier=='external':e.external_emergency=True
  elif barrier=='red':e.light='RED';e.visual_go=False
  elif barrier=='official':e.require_official=True;e.official=False
  else:e.controller_ready=False
  assert not e.recovery_step(t+.05,ctx)['enabled']


def test_reverse_bad_frames_confirm_while_previous_forward_target_remains_bounded():
 e,c,r,t=committed()
 assert lost(e,r,t+.03,True) is not None
 assert lost(e,r,t+.06,True) is not None
 assert lost(e,r,t+.09,True) is None
 assert r.stop_reason=='reverse_risk_persistent'


def test_motion_compensation_reselects_positive_targets_instead_of_freezing_one_point():
 e,c,r,t=committed();old=r.memory['paths']['selected'].copy();ctx=lost(e,r,t+.15)
 assert ctx is not None and not np.array_equal(old,ctx['paths']['selected'])
 assert ctx['target'][0]>0 and not np.allclose(ctx['target'],r.memory['target'])
 with pytest.raises(ValueError):transformed_paths(r.memory['paths'],(100.,0.,0.))


def test_no_permanent_latch_after_three_fresh_accepted_frames():
 e,c,r,t=committed();lost(e,r,t+.51);assert r.expired
 e.path_confirmed=False;e.ready_count=0
 for i in range(1,4):
  n=t+.52+i*.03;p,l,right=geometry();r.health(True,n)
  e.camera(p,l,right,1.,(True,True),n,n,stamp=n);e.lidar([],n,stamp=n)
  r.note_input(n,n,e.camera_stamp==n,1.,(True,True),e.last_camera_issue)
  context=r.prepare(n,n,0.,0.)
  if i<3:assert context is None and r.stop_reason
  else:
   assert not r.stop_reason
   out=c.constrain(e.step(n),n,0.);assert out['enabled']
   r.observe_safe(out,c,e,n,n);assert not r.expired


def test_single_opposite_direction_jump_is_rejected_then_existing_smoothing_returns():
 e,c,r,t=committed();p,l,right=geometry(-1.)
 e.camera(p,l,right,1.,(True,True),t+.03,t+.03,stamp=t+.03)
 assert e.camera_stamp==t and e.last_camera_issue=='trajectory_jump'
 r.health(True,t+.03);r.note_input(t+.03,t+.03,False,1.,(True,True),'trajectory_jump')
 ctx=r.prepare(t+.03,t+.03,1.2,.08);assert ctx and r.direction=='LEFT'
 p,l,right=geometry();p[:,1]+=.10;l[:,1]+=.10;right[:,1]+=.10
 e.camera(p,l,right,1.,(True,True),t+.06,t+.06,stamp=t+.06)
 assert e.camera_stamp==t+.06
 assert 0<e.path[0,1]-.175*.1*.1<.10  # Existing alpha, no duplicate blender.


def test_replay_and_paused_ros_clock_do_not_extend_hold():
 e,c,r,t=committed();ctx=lost(e,r,t+.1);receiver=RecoveryReceiver()
 receiver.health(True,t+.1);data=wire_context(ctx)
 assert receiver.update(data,t+.1,t+.1)
 receiver.health(True,t+.55)
 assert not receiver.update(data,t+.1,t+.55)
 assert receiver.context(t+.1,t+.55) is None


def test_lidar_uses_predicted_camera_corridor_without_overwriting_measurement_time():
 e,c,r,t=committed();p,l,right=geometry();f=CameraWallFilter(.24)
 f.update_camera(p,l,right,1.,(True,True),t,t)
 ctx=lost(e,r,t+.3);receiver=RecoveryReceiver();receiver.health(True,t+.3)
 assert receiver.update(wire_context(ctx),t+.3,t+.3)
 f.recovery_context=receiver.wall_context(t+.3,t+.3)
 ready,candidates,d=f.classify(np.empty((0,2)),np.empty(0,dtype=int),t+.3,t+.3)
 assert ready and d['state']=='CORNER_PATH_HOLD_CORRIDOR'
 assert f.camera['stamp']==t and d['trajectory_origin_stamp']==t
 f.recovery_context=receiver.wall_context(t+.61,t+.61)
 assert not f.classify(np.empty((0,2)),np.empty(0,dtype=int),t+.61,t+.61)[0]


def test_uncommitted_straight_never_gets_previous_path_driving_permission():
 r=CornerRecovery();r.health(True,1.)
 assert r.prepare(1.,1.,7.,0.) is None and r.memory is None


def test_low_confidence_frame_uses_safe_memory_without_refreshing_it():
 e,c,r,t=committed();p,l,right=geometry()
 e.camera(p,l,right,.45,(True,False),t+.03,t+.03,stamp=t+.03)
 r.health(True,t+.03);r.note_input(t+.03,t+.03,True,.45,(True,False),'low_path_confidence')
 ctx=r.prepare(t+.03,t+.03,1.2,.08)
 assert ctx and ctx['source_stamp']==t and e.recovery_step(t+.03,ctx)['enabled']


def test_camera_health_required_even_when_path_packet_is_fresh():
 e,c,r,t=committed();r.health(False,t+.01)
 assert r.prepare(t+.02,t+.02,1.2,.08) is None
 assert r.stop_reason=='camera_timeout' and r.expired


def test_observed_vehicle_outside_corridor_stops_without_using_old_geometry():
 e,c,r,t=committed()
 r.note_input(t+.03,t+.03,True,1.,(True,True),'',corridor_ok=False);r.health(True,t+.03)
 assert r.prepare(t+.03,t+.03,1.2,.08) is None
 assert r.stop_reason=='corridor_lost'


def test_corner_exit_requires_stable_low_geometry_then_returns_straight():
 e,c,r,t=committed();x=np.arange(.1,5.01,.1);p=np.column_stack((x,np.zeros_like(x)));states=[]
 for i in range(1,14):
  c.update(p,t+i*.03,t+i*.03,speed=1.2);states.append(c.state)
 assert 'CORNER_EXIT' in states and states[-1]=='STRAIGHT'


def test_rejected_corner_frame_does_not_renew_geometry_source_stamp():
 e,c,r,t=committed();c.reject(t+.03)
 assert c.policy(t+.03,1.2)['camera_source_stamp']==t


def test_steep_corner_walls_do_not_invalidate_a_valid_drive_path():
 e=DecisionEngine(vehicle_width=.24,wheelbase=.34,cruise=7.,require_official=False)
 x=np.arange(.1,2.71,.1);radius=1/.35;p=np.column_stack((x,radius-np.sqrt(radius*radius-x*x)))
 offset=np.column_stack((np.zeros(len(p)),1.2*np.sqrt(1+np.gradient(p[:,1],p[:,0])**2)))
 left,right=p+offset,p-offset
 from team_01_race.navigation import valid_boundary
 assert valid_path(p) and valid_boundary(left) and valid_boundary(right)
 assert not valid_path(left)  # The steering path restriction is deliberately unchanged.
 for i in range(3):e.camera(p,left,right,1.,(True,True),i,1+i*.03,stamp=1+i*.03)
 assert e.camera_stamp==1.06 and e.last_camera_issue==''
 left[2,1]=math.nan
 e.camera(p,left,right,1.,(True,True),4,1.1,stamp=1.1)
 assert e.last_camera_issue=='boundary_invalid' and e.camera_stamp==1.06


def test_selection_failure_can_hold_even_when_camera_packet_was_valid():
 e,c,r,t=committed();r.health(True,t+.03);e.lidar([],t+.03,stamp=t+.03)
 assert r.input_good
 ctx=r.prepare(t+.03,t+.03,1.2,.08,force_hold=True)
 assert ctx and ctx['source_stamp']==t and e.recovery_step(t+.03,ctx)['enabled']


def test_actual_body_twist_moves_cached_geometry_without_reading_pose():
 e,c,r,t=committed();r.integrate(t+.1,7.,0.,twist=(1.,.5,.4))
 assert r.motion[0]>0 and r.motion[1]>.04 and r.motion[2]==pytest.approx(.04)
 assert valid_path(r.prediction()['selected'])


def test_temporal_extension_of_a_moved_straight_does_not_create_curvature():
 from team_01_race.navigation import temporal_y,path_geometry
 xs=np.arange(.1,5.01,.1);old=np.column_stack((xs[2:],.2*xs[2:]+.3))
 merged=np.column_stack((xs,temporal_y(xs,old)))
 assert np.max(np.abs(path_geometry(merged)[1]))<1e-10

@pytest.mark.parametrize('source',['CAMERA_PATH','LIDAR_PATH','FUSED_PATH'])
def test_general_previous_path_preserves_source_age_and_sensor_health(source):
 from team_01_race.corner_recovery import RecoveryConfig
 e=DecisionEngine(vehicle_width=.24,wheelbase=.34,cruise=7.,require_official=False)
 e.set_light('LIGHTS_OUT');e.controller_ready=True;e.actual_speed=1.2;e.path_source=source
 c=CornerDetector();r=CornerRecovery(RecoveryConfig(allow_general_path_hold=True))
 x=np.arange(.1,5.01,.1);p=np.column_stack((x,x*0))
 for i in range(5):
  t=1+i*.03;e.camera(p,p+[0,1.2],p-[0,1.2],1.,(True,True),t,t,stamp=t)
  e.lidar([],t,stamp=t);r.health(source=='CAMERA_PATH',t);r.lidar_health(True,t)
  r.note_input(t,t,True,1.,(True,True),'');c.update(e.path,t,t,speed=1.2)
  r.observe_safe(c.constrain(e.step(t),t,1.2),c,e,t,t)
 assert r.memory is not None and r.memory['severity']=='STRAIGHT'
 for dt in (.05,.2,.45):
  now=t+dt;r.health(source=='CAMERA_PATH',now);r.lidar_health(True,now)
  r.note_input(now,now,False,0.,(False,False),'path_invalid');e.lidar([],now,stamp=now)
  context=r.prepare(now,now,1.2,0.,(1.2,0.,0.))
  assert context and context['phase']=='PATH_HOLD' and context['source_sensor']==source
  assert e.recovery_step(now,context)['speed']<=1.2
  receiver=RecoveryReceiver(r.c);receiver.health(source=='CAMERA_PATH',now);receiver.lidar_health(True,now)
  receiver.update(wire_context(context),now,now)
  assert receiver.context(now,now) is not None
 now=t+.51;r.health(source=='CAMERA_PATH',now);r.lidar_health(True,now)
 assert r.prepare(now,now,1.2,0.) is None and r.stop_reason=='path_lost_timeout'


def test_memory_width_supports_different_center_and_boundary_sample_counts():
 e,c,r,t=committed();p,l,right=geometry();r.health(True,t+.03)
 e.camera(p[:25],l,right,1.,(True,True),t+.03,t+.03,stamp=t+.03);e.lidar([],t+.03,stamp=t+.03)
 r.note_input(t+.03,t+.03,True,1.,(True,True),'');c.update(e.path,t+.03,t+.03,speed=1.2)
 out=c.constrain(e.step(t+.03),t+.03,1.2);r.observe_safe(out,c,e,t+.03,t+.03)
 assert r.memory is not None and .55<=r.track_width<=4.
