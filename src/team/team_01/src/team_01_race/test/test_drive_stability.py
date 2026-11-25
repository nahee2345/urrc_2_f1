import math
from types import SimpleNamespace
import numpy as np
from team_01_race.depth_corner import DepthCornerTrack
from team_01_race.speed_authority import SpeedIncreaseConfirmation,safety_decrease_priority
from team_01_race.drive_stability import DriveStability,reduction_reason

SPEEDS=[10.,7.,5.5,4.5]
def frame(t,level=0,valid=True):return dict(stamp=t,depth_valid=valid,confidence=.9,depth_corner_candidate=level>0,level=level,direction=1,distance=12. if level else None,straight_geometry=level==0,depth_supported_distance_m=16.,depth_curvature_supported_distance_m=15.)
def update(tr,t,level=0,valid=True,lidar=True):return tr.update(frame(t,level,valid),t,t,0.,True,lidar)
def policy(tr,t,odom=True):return tr.policy(t,0.,odom,7.,SPEEDS,4.,.75,.1)
def straight():
 tr=DepthCornerTrack()
 for t in (.1,.2,.3,.4):update(tr,t)
 return tr

def test_one_candidate_does_not_revoke_confirmed_straight():
 tr=straight();update(tr,.5,3)
 assert not tr.latched and policy(tr,.5)['depth_corner_speed_limit']==10.
 update(tr,.6)
 assert policy(tr,.6)['depth_corner_severity']=='STRAIGHT'
 for t in (.7,.8,.9):update(tr,t,3)
 assert tr.latched and policy(tr,.9)['depth_corner_speed_limit']==4.5

def test_invalid_notification_retains_only_original_fresh_observation():
 tr=straight();stamp=tr.time
 update(tr,.5,valid=False)
 assert tr.time==stamp and policy(tr,.5)['depth_corner_speed_limit']==10.
 update(tr,.6,valid=False)
 assert tr.time==stamp
 assert policy(tr,.8)['depth_target_speed']<=3.5
 update(tr,.9,valid=False)
 assert not tr.straight

def test_odom_availability_does_not_reclassify_fresh_straight_geometry():
 tr=straight()
 assert policy(tr,.4,False)['depth_corner_severity']=='STRAIGHT'
 for t in (.5,.6,.7):update(tr,t,3)
 assert tr.latched and policy(tr,.7,False)['depth_target_speed']<=3.5

def test_straight_exit_still_requires_passage_and_lidar_agreement():
 tr=straight()
 for t in (.5,.6,.7):update(tr,t,3)
 for t in (.8,.9,1.,1.1):tr.update(frame(t),t,t,13.,True,False)
 assert tr.latched
 tr.update(frame(1.2),1.2,1.2,13.,True,True)
 assert tr.latched is None
 tr.update(frame(1.3),1.3,1.3,13.,True,True)
 assert tr.policy(1.3,13.,True,4.,SPEEDS,4.,.75,.1)['depth_corner_speed_limit']==10.

def test_speed_increase_requires_distinct_observations_not_timer_ticks():
 gate=SpeedIncreaseConfirmation(3)
 assert gate.update(3.5,1,'DEPTH_UNAVAILABLE')==(3.5,False)
 for _ in range(20):assert gate.update(10.,1,'CRUISE')==(3.5,True)
 for key in (2,3):assert gate.update(10.,key,'CRUISE')==(3.5,True)
 assert gate.update(10.,4,'CRUISE')==(10.,False)
 assert gate.update(4.5,4,'DEPTH_CORNER')==(4.5,False)
 for key in (5,6):assert gate.update(10.,key,'CRUISE')==(4.5,True)
 assert gate.update(3.5,6,'DEPTH_UNAVAILABLE')==(3.5,False)

def test_verified_priority_does_not_depend_on_delayed_feedback_flag():
 assert safety_decrease_priority(4.167,2.548,2.548,10.,10.,command_fresh=True,limiter='DEPTH_CORNER')
 assert safety_decrease_priority(6.,2.,10.,2.,10.,command_fresh=False,limiter='')
 assert not safety_decrease_priority(4.,10.,10.,10.,10.,command_fresh=True,limiter='CRUISE')
 assert not safety_decrease_priority(6.,4.,4.,10.,10.,command_fresh=False,limiter='DEPTH_CORNER')

def test_history_alignment_uses_one_geometry_time_in_both_directions():
 from team_01_race.navigation import DecisionEngine
 from team_01_race.decision_node import DecisionNode
 from team_01_race.corner_recovery import transformed_paths
 x=np.arange(.1,5.01,.1);p=np.column_stack((x,np.full(len(x),.2)))
 walls=dict(center=p,left=p+[0,1.2],right=p-[0,1.2],selected=p)
 engine=DecisionEngine(vehicle_width=.24,wheelbase=.26);engine.camera(p,walls['left'],walls['right'],1.,(True,True),1.,1.,stamp=1.)
 twist=(4.,0.,.3);engine.last_selected=p.copy();engine.update_motion(1.,twist);engine.update_motion(1.15,twist)
 selected_now=engine.last_selected.copy()
 node=SimpleNamespace(engine=engine,geometry_projection_time=1.,velocity=SimpleNamespace(fresh=lambda _:True,linear=(4.,0.,0.),yaw_rate=.3),now_seconds=lambda:1.15,recovery=SimpleNamespace(last_context=None))
 DecisionNode.align_history_to_capture(node,1.1)
 angle=.03;expected=transformed_paths(walls,(4*math.sin(angle)/.3,4*(1-math.cos(angle))/.3,angle))['center']
 sample=np.arange(.5,4.4,.1)
 assert np.max(abs(np.interp(sample,engine.path[:,0],engine.path[:,1])-np.interp(sample,expected[:,0],expected[:,1])))<1e-5
 assert engine.geometry_motion_time==1.1
 assert np.array_equal(engine.last_selected,selected_now)
 # Same-source second call must not move history twice.
 old=engine.path.copy();DecisionNode.align_history_to_capture(node,1.1)
 assert np.array_equal(old,engine.path)

def test_same_cycle_diagnostic_request_is_actual_decision_not_cruise():
 path=np.column_stack((np.arange(.1,5.01,.1),np.zeros(50)))
 r=DriveStability().snapshot(dict(requested_speed=10.,depth_corner_severity='STRAIGHT'),path,stamp=1.,requested=2.548,final_target=2.548,final=2.548,actual=4.167,braking=True,clamp=True,limiter='DEPTH_CORNER',stop_reason='',source='LIDAR_PATH',desired_lookahead=1.5,effective_lookahead=1.4,lookahead_speed=4.167,target=path[15],raw_steering=0.,steering=0.)
 assert r['requested_speed']==2.548 and r['braking_invariant_ok']
 assert r['braking_reason']=='DEPTH_CORNER' and r['selected_steering_source']=='LIDAR_PATH'

def test_delayed_or_reordered_intrinsics_do_not_invalidate_fresh_depth():
 from sensor_msgs.msg import CameraInfo,Image
 from team_01_race.depth_node import DepthNode
 from team_01_race.depth_geometry import DepthConfig
 from team_01_race.corner import CornerConfig
 from test_depth_corner import render_wall_depth
 x=np.arange(.1,22.,.25);d,k,pose=render_wall_depth(np.column_stack((x,np.zeros(len(x)))))
 info=CameraInfo(width=640,height=480,k=k);info.header.frame_id='team_01/camera_optical_frame';info.header.stamp.sec=100
 state=SimpleNamespace(info=None)
 DepthNode.on_info(state,info);assert state.info is info
 old=CameraInfo(width=640,height=480,k=k);old.header.frame_id=info.header.frame_id;old.header.stamp.sec=99
 DepthNode.on_info(state,old);assert state.info is info
 invalid=CameraInfo();invalid.header.stamp.sec=102
 DepthNode.on_info(state,invalid);assert state.info is info
 packets=[]
 state.c=DepthConfig();state.pose=pose;state.ground_z=-.085;state.corner=CornerConfig()
 state.last_stamp=state.last=-math.inf;state.near=.03;state.far=50.
 state.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=101000000000))
 state.pub=SimpleNamespace(publish=lambda m:packets.append(m))
 msg=Image(width=640,height=480,encoding='32FC1',step=640*4,data=d.astype('<f4').tobytes())
 msg.header.frame_id=info.header.frame_id;msg.header.stamp.sec=101
 DepthNode.on_depth(state,msg)
 import json
 assert json.loads(packets[-1].data)['depth_valid']
 # A genuinely stale Image is still rejected; cached K never refreshes it.
 msg.header.stamp.sec=100;DepthNode.on_depth(state,msg);assert len(packets)==1

def test_real_control_node_final_braking_ceiling_with_and_without_feedback_flag():
 import os,json,rclpy
 os.environ['ROS_DOMAIN_ID']='169'
 from std_msgs.msg import String
 from team_01_race.control_node import PurePursuitControlNode
 rclpy.init(args=['--ros-args','-p','use_sim_time:=false'])
 node=PurePursuitControlNode()
 p=np.column_stack((np.arange(.1,5.01,.1),np.zeros(50)))
 try:
  for flag in (True,False):
   now=node.now_seconds();stamp=node.get_clock().now().to_msg();source=stamp.sec+stamp.nanosec*1e-9
   node.guard.camera_geometry(p,p+[0,1.2],p-[0,1.2],1.,source,now)
   node.guard.set_severity('STRAIGHT',7.,8.,now)
   command=dict(stamp=source,speed=2.548,enabled=True,path_source='LIDAR_PATH',mode='CENTER',reason='fixture',corner_braking=flag,
    speed_trace=dict(planner_limiter='DEPTH_CORNER'),trajectory=p.tolist(),trajectory_header=dict(frame_id='team_01/base_link',sec=stamp.sec,nanosec=stamp.nanosec))
   node.on_decision_command(String(data=json.dumps(command)))
   node.speed=4.167;node.last_time=node.now_seconds()-.02;node.tick()
   assert node.enabled and node.speed<=2.548+1e-9
  stamp=node.get_clock().now().to_msg();now=node.now_seconds()
  node.guard.camera_geometry(p,p+[0,1.2],p-[0,1.2],1.,stamp.sec+stamp.nanosec*1e-9,now)
  node.guard.set_severity('STRAIGHT',7.,8.,now)
  command.update(stamp=stamp.sec+stamp.nanosec*1e-9,speed=7.,corner_braking=False,speed_trace=dict(planner_limiter='CRUISE'),trajectory_header=dict(frame_id='team_01/base_link',sec=stamp.sec,nanosec=stamp.nanosec))
  node.on_decision_command(String(data=json.dumps(command)));node.speed=4.;node.last_time=node.now_seconds()-.02;node.tick()
  assert 4.<node.speed<4.1  # Ordinary acceleration smoothing still applies.
  # The body has not instantaneously slowed to a lower wheel command.
  stamp=node.get_clock().now().to_msg();now=node.now_seconds()
  node.velocity.update(stamp.sec+stamp.nanosec*1e-9,now,0.,(5.,0.,0.))
  packets=[];node.stability_pub=SimpleNamespace(publish=lambda m:packets.append(json.loads(m.data)))
  node.speed=2.;node.lookahead=None;node.last_time=now-.02;node.tick()
  assert packets[-1]['lookahead_speed_source']=='odometry_twist'
  assert packets[-1]['lookahead_speed']==5.
  assert abs(packets[-1]['desired_lookahead']-2.25)<1e-6
  # Independent Recovery diagnostics may arrive before/after the matching
  # command. They cannot label a CENTER speed as a passing speed.
  recovery_packets=[]
  node.recovery_control_pub=SimpleNamespace(publish=lambda m:recovery_packets.append(json.loads(m.data)))
  node.recovery_debug=dict(state='RIGHT_PASS',path_source='PREVIOUS_LIDAR_PATH')
  node.tick()
  assert recovery_packets[-1]['state']==command['mode']=='CENTER'
  assert recovery_packets[-1]['path_source']==command['path_source']=='LIDAR_PATH'
  assert recovery_packets[-1]['decision_command_stamp']==command['stamp']
 finally:node.destroy_node();rclpy.shutdown()

def test_confirmed_straight_hysteresis_band_does_not_reopen_on_single_noise():
 tr=straight()
 d=frame(.5);d.update(straight_geometry=False,heading_change_deg=7.,segments=[dict(p90_curvature=.07)])
 tr.update(d,.5,.5,0.,True,True)
 assert policy(tr,.5)['depth_corner_severity']=='STRAIGHT'
 # A new unknown scene cannot bootstrap a straight from the hysteresis band.
 unknown=DepthCornerTrack();unknown.update(d,.5,.5,0.,True,True)
 assert policy(unknown,.5)['depth_corner_severity']=='UNKNOWN'
 # Nor may an unsupported/noisy non-straight LiDAR scene renew it.
 tr.update(dict(d,stamp=.6),.6,.6,0.,True,False)
 assert policy(tr,.6)['depth_target_speed']<=3.5

def test_componentwise_projection_matches_original_matrix_rotation():
 from team_01_race.depth_geometry import project_depth,rotation
 depth=np.arange(1.,49.).reshape(6,8);k=np.array([[123.,0,4.],[0,124.,3.],[0,0,1.]])
 pose=[.23,.01,.08,.04,.21,-.03];v,u=np.indices(depth.shape)
 optical=np.column_stack((((u-4)*depth/123).ravel(),((v-3)*depth/124).ravel(),depth.ravel()))
 reference=optical@(rotation(*pose[3:])@rotation(-math.pi/2,0,-math.pi/2)).T+pose[:3]
 assert np.allclose(project_depth(depth,k,pose,.03,50),reference,atol=1e-12)

def test_batched_depth_fits_match_scalar_lidar_fits_without_threshold_changes():
 from team_01_race.lidar_track import LidarTrack
 rng=np.random.default_rng(45)
 for irregular in (False,True):
  x=np.arange(0.,18.,.1)
  if irregular:x=np.delete(x,np.arange(7,175,11))
  values=np.column_stack((.02*x*x+.01*rng.normal(size=len(x)),np.sin(x/5)))
  scalar=LidarTrack()._smooth(x,values,x,7)
  fast=LidarTrack(batch_smoothing=True)._smooth(x,values,x,7)
  assert np.allclose(scalar,fast,atol=1e-10)

def test_primary_lidar_one_stability_glitch_is_not_a_new_fallback_source():
 from team_01_race.speed_authority import PrimaryLidarAdmission
 gate=PrimaryLidarAdmission(3)
 for i in range(3):gate.update(i,True,True)
 assert gate.confirmed
 assert gate.update(3,True,False)
 assert gate.update(3,True,False)  # Timer/replayed key cannot confirm degradation.
 assert gate.update(4,True,True)
 assert gate.update(5,True,False)
 assert not gate.update(6,True,False)
 for i in (7,8,9):gate.update(i,True,True)
 assert gate.confirmed
 assert not gate.update(10,False,True)  # Single wall/low confidence/invalid remains immediate.

def test_unconfirmed_path_hint_does_not_become_a_latched_braking_ceiling():
 gate=SpeedIncreaseConfirmation(3)
 gate.update(7.,1,'CRUISE')
 assert gate.update(4.5,2,'PATH_CURVATURE',verified_decrease=False)==(4.5,False)
 assert gate.limit==7.
 assert gate.update(7.,3,'CRUISE')==(7.,False)
 assert gate.update(4.5,4,'DEPTH_CORNER',verified_decrease=True)==(4.5,False)
 assert gate.limit==4.5

def test_confirmed_curve_cannot_keep_straight_authority_when_odom_unavailable():
 tr=straight()
 for t in (.5,.6,.7):tr.update(frame(t,3),t,t,0.,False,True)
 assert not tr.straight and tr.latched is None
 assert policy(tr,.7,False)['depth_target_speed']<=4.5

def test_preview_hold_reuses_only_finite_observed_space_and_original_source_time():
 from team_01_race.speed_authority import ObservedPreviewHold
 from team_01_race.depth_corner import CornerTravel
 odom=CornerTravel();odom.update(1.,1.,0.,(0.,0.,0.),6.);odom.update(1.1,1.1,0.,(.6,0.,0.),6.)
 assert abs(odom.distance_at(1.05)-.3)<1e-9
 assert odom.distance_at(.9) is None and odom.distance_at(1.2) is None
 hold=ObservedPreviewHold(.5);hold.record(1.05,1.05,odom.distance_at(1.05),odom.epoch,17.)
 args=dict(odom_fresh=True,lidar_straight=True,command=6.,decel=2.5,margin=1.35)
 h=hold.evaluate(1.42,2.52,odom.epoch,**args)
 assert h and h['limit']==6. and abs(h['remaining_m']-14.78)<1e-9 and h['source_stamp']==1.05
 # A stale preview never opens acceleration authority even if command feedback rises.
 assert hold.evaluate(1.45,2.7,odom.epoch,**dict(args,command=7.))['limit']==6.
 limited=hold.evaluate(1.46,14.,odom.epoch,**args)
 assert limited['limit']<3.5  # Original measured space is almost consumed.
 assert hold.evaluate(1.56,2.52,odom.epoch,**args) is None
 assert hold.evaluate(1.42,2.52,odom.epoch+1,**args) is None
 assert hold.evaluate(1.42,2.52,odom.epoch,**dict(args,odom_fresh=False)) is None
 assert hold.evaluate(1.42,2.52,odom.epoch,**dict(args,lidar_straight=False)) is None
 hold.clear();assert hold.evaluate(1.42,2.52,odom.epoch,**args) is None


def test_odom_reset_invalidates_preview_interpolation_epoch():
 from team_01_race.depth_corner import CornerTravel
 o=CornerTravel();o.update(.1,.1,0,(0,0,0),1.);o.update(.2,.2,0,(.1,0,0),1.)
 assert o.distance_at(.15) is not None
 assert not o.update(.3,.3,0,(10,0,0),1.)
 assert o.epoch==1 and o.distance_at(.15) is None


def test_stronger_brake_command_does_not_shorten_unobserved_body_braking_forecast():
 import rclpy
 from team_01_race.control_node import PurePursuitControlNode
 rclpy.init(args=['--ros-args','-p','use_sim_time:=false',
  '-p','control.max_accel_mps2:=3.5','-p','control.max_decel_mps2:=3.5',
  '-p','safety.assumed_max_decel_mps2:=2.5'])
 node=PurePursuitControlNode()
 try:
  assert node.accel==node.decel==3.5
  assert node.assumed_decel==node.guard.brake_decel==2.5
  node.now_seconds=lambda:100.
  node.last_time=99.98;node.motion_speed_bound=10.;node.tick()
  # Without fresh Odom, a stronger command is not evidence of stronger braking.
  assert abs(node.motion_speed_bound-9.95)<1e-8
 finally:node.destroy_node();rclpy.shutdown()
