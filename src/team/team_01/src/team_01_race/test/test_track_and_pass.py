import sys,math
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.lidar_track import LidarTrack
from team_01_race.path_manager import SourceSelector,geometry_valid
from team_01_race.navigation import Obstacle,pure_pursuit,valid_path,DecisionEngine
from team_01_race.obstacle_pass import ObstaclePassPlanner,inside_corridor
from team_01_race.corridor_geometry import basis,footprint_distance


def corridor(curve=0.,half=1.2):
 x=np.arange(.1,5.01,.1);p=np.column_stack((x,curve*x*x));_,n=basis(p)
 # true normal boundaries, separately resampled at common X for planning
 from team_01_race.corridor_geometry import resample_x,same_x
 l=resample_x(p+half*n);r=resample_x(p-half*n);a=same_x([p,l,r]);return a

def laser_walls(p,l,r):
 # Ordered independent wall returns; preserve raw beam connectivity ids.
 a=[];idx=[]
 for i,q in enumerate((l,r[::-1])):
  dense=np.arange(q[:,0].min(),q[:,0].max(),.025);points=np.column_stack((dense,np.interp(dense,q[::-1,0] if i else q[:,0],q[::-1,1] if i else q[:,1])))
  if i:points=points[::-1]
  a.append(points);idx.extend(np.arange(len(points))+sum(len(v)+20 for v in a[:-1]))
 return np.concatenate(a),np.array(idx)

def test_lidar_curved_prefix_single_wall_and_expiry():
 p,l,r=corridor(.10);q,idx=laser_walls(p,l,r);track=LidarTrack()
 both=track.update(q,idx,1.,1.,p)
 assert geometry_valid(both),track.debug
 assert both['path'][-1,0]<=min(l[-1,0],r[-1,0])+.01
 n=len(q)//2
 one=track.update(q[:n],idx[:n],1.04,1.04,p)
 assert geometry_valid(one),track.debug
 assert one['confidence']<both['confidence']
 assert track.update(q[:n],idx[:n],3.2,3.2,p) is None


def test_source_switch_counts_new_frames_not_replays():
 p,l,r=corridor();c=dict(path=p,left=l,right=r,confidence=1.,observed=(True,True));d=dict(c,confidence=.8)
 s=SourceSelector()
 assert s.select(c,d,dict(camera=1.,lidar=1.))[0] is None
 assert s.select(c,d,dict(camera=1.,lidar=1.))[0] is None
 assert s.select(c,d,dict(camera=1.05,lidar=1.05))[0] is None
 assert s.select(c,d,dict(camera=1.10,lidar=1.10))[1]=='CAMERA_PATH'
 assert s.select(None,d,dict(camera=1.15,lidar=1.15))[1]=='LIDAR_PATH'
 for _ in range(5):assert s.select(c,d,dict(camera=1.5,lidar=1.5))[1]=='LIDAR_PATH'
 assert s.select(c,d,dict(camera=1.55,lidar=1.55))[1]=='LIDAR_PATH'
 assert s.select(c,d,dict(camera=1.60,lidar=1.60))[1]=='LIDAR_PATH'
 assert s.select(c,d,dict(camera=1.70,lidar=1.70))[1]=='CAMERA_PATH'


def test_normal_offset_curved_pass_and_full_footprint_gap():
 p,l,r=corridor(.06);planner=ObstaclePassPlanner(.242,.34)
 o=Obstacle('car',3.4,float(np.interp(3.4,p[:,0],p[:,1])),.30,.4,1.)
 q,gap,offset=planner.candidate(p,l,r,o,'LEFT_PASS')
 assert q is not None and gap>.542
 assert footprint_distance(q,o)>=.271-1e-5
 assert inside_corridor(q,l,r,.271)
 # Normal displacement changes X as well as Y on a curve.
 assert np.max(abs(q[:,1]-np.interp(q[:,0],p[:,0],p[:,1])))>0.3
 assert valid_path(q) and pure_pursuit(q,2.,.34)[1][0]>0
 wide=Obstacle('wide',3.4,o.y,1.8,.8,1.)
 assert planner.candidate(p,l,r,wide,'LEFT_PASS')[0] is None
 assert planner.candidate(p,l,r,wide,'RIGHT_PASS')[0] is None


def test_left_right_no_flip_and_measured_motion_merge():
 p,l,r=corridor();planner=ObstaclePassPlanner(.242,.34)
 o=Obstacle('car',3.8,-.6,.30,.3,1.)
 q,mode,_=planner.plan(p,l,r,[o],[o],1.,1.)
 assert mode=='LEFT_PASS' and q is not None
 o.y=-.58
 assert planner.plan(p,l,r,[o],[o],2.,1.04)[1]=='LEFT_PASS'
 # Measured ego translation proves the rear is behind; one empty frame cannot.
 planner.integrate(1.,(2.,0.,0.))
 for i in range(12):planner.integrate(1.+(i+1)*.2,(2.,0.,0.))
 for i in range(3):q,mode,_=planner.plan(p,l,r,[],[],3+i,3.5+i*.04)
 assert mode=='MERGE_CENTER'
 for i in range(3):planner.note_selected(p,p,10+i)
 assert planner.state=='CENTER'
 right=Obstacle('car2',3.8,.6,.3,.3,1.)
 assert ObstaclePassPlanner(.242,.34).plan(p,l,r,[right],[right],1.,1.)[1]=='RIGHT_PASS'


def test_invalid_geometry_debug_bool_and_unseen_extension():
 import json
 from team_01_race.path_manager import geometry_valid
 p,l,r=corridor();bad=dict(path=p.copy(),left=l,right=r,confidence=1.)
 bad['path'][:,1]=np.arange(len(p))
 assert geometry_valid(bad) is False
 json.dumps(dict(valid=geometry_valid(bad)))
 e=DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=False)
 short=p[p[:,0]<=2.]
 e.camera(short,short+[0,1.2],short-[0,1.2],1.,(True,True),1,1.,stamp=1.)
 new=p.copy();new[:,1]=.1*np.maximum(0,new[:,0]-2.)**2
 e.camera(new,new+[0,1.2],new-[0,1.2],1.,(True,True),2,1.04,stamp=2.)
 assert e.camera_stamp==2 and e.last_camera_issue==''
 assert e.path[-1,1]==new[-1,1]


def test_real_gazebo_bend_scans_keep_a_valid_observed_prefix():
 import json
 from team_01_race.lidar_filter import scan_points
 fixtures=json.loads((Path(__file__).parent/'fixtures/curved_wall_scans.json').read_text())
 track=LidarTrack();track.note_width(2.4,0.,fixtures[0]['stamp'])
 for f in fixtures:
  valid,q,ids=scan_points(f['ranges'],f['angle_min'],f['angle_increment'],f['range_min'],f['range_max'],np.eye(3),[.195,0,.02])
  result=track.update(q,ids,f['stamp'],f['stamp'])
  assert valid and geometry_valid(result),track.debug
  assert track.debug['forward_observed_length']>=1.
  assert result['path'][-1,0]<=q[:,0].max()
 assert result['confidence']<.9


def test_camera_fused_lidar_camera_switches_preserve_common_geometry():
 p,l,r=corridor();camera=dict(path=p,left=l,right=r,confidence=1.,observed=(True,True));laser=dict(camera,confidence=.8)
 router=SourceSelector();selected=[]
 for frames,current,desired in [([1.,1.1,1.2],camera,'CAMERA_PATH'),
        ([1.3,1.4,1.5,1.6,1.7,1.8],dict(camera,confidence=.68),'FUSED_PATH'),
        ([2.0],None,'LIDAR_PATH'),([2.4,2.5,2.6],camera,'CAMERA_PATH')]:
  for stamp in frames:geometry,source=router.select(current,laser,dict(camera=stamp,lidar=stamp))
  assert source==desired
  selected.append(geometry['path'])
 for previous,current in zip(selected,selected[1:]):assert np.max(abs(current[:,1]-np.interp(current[:,0],previous[:,0],previous[:,1])))<1e-6


def test_curved_corridor_support_uses_normals_beyond_inner_wall_x_end():
 from team_01_race.corridor_geometry import resample_x,supported_prefix
 theta=np.linspace(.04,1.3,180);radius=1/.35
 center=np.column_stack((radius*np.sin(theta),radius*(1-np.cos(theta))))
 left=np.column_stack(((radius-1.2)*np.sin(theta),radius-(radius-1.2)*np.cos(theta)))
 right=np.column_stack(((radius+1.2)*np.sin(theta),radius-(radius+1.2)*np.cos(theta)))
 center=resample_x(center);left=resample_x(left);right=resample_x(right)
 supported=supported_prefix(center,left,right)
 assert supported[-1,0]>left[-1,0]+.5
 assert inside_corridor(supported,left,right,.271)
 outside=supported.copy();outside[:,1]+=2.
 assert not inside_corridor(outside,left,right,.271)


def test_reacquired_obstacle_id_keeps_active_pass_side():
 p,l,r=corridor();planner=ObstaclePassPlanner(.242,.34)
 old=Obstacle('old',3.8,-.6,.3,.3,1.)
 path,mode,_=planner.plan(p,l,r,[old],[old],1.,1.)
 assert mode=='LEFT_PASS'
 replacement=Obstacle('new',3.75,-.59,.3,.3,1.)
 path,mode,_=planner.plan(p,l,r,[replacement],[replacement],2.,1.04)
 assert mode=='LEFT_PASS' and planner.observation.id=='new'
 assert 'RIGHT_PASS' not in planner.transitions


def test_motion_compensation_preserves_source_age_and_frame_continuity():
 p,l,r=corridor();e=DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=False)
 e.controller_ready=True;e.set_light('LIGHTS_OUT')
 for i in range(3):e.camera(p,l,r,1.,(True,True),i,i*.05,stamp=i*.05)
 original_time=e.camera_time;original_stamp=e.camera_stamp
 e.update_motion(.2,(2.,0.,.2))
 assert e.camera_time==original_time and e.camera_stamp==original_stamp
 assert np.median(e.path[:,1])<-.02
 e.camera(e.path.copy(),e.left.copy(),e.right.copy(),1.,(True,True),4,.2,stamp=.2)
 assert e.last_camera_issue=='' and e.path_confirmed


def test_long_steep_wall_is_not_rejected_for_short_x_projection():
 x=np.linspace(.2,.95,100);wall=np.column_stack((x,1.2+2*x))
 track=LidarTrack();track.note_width(2.4,0.,1.)
 result=track.update(wall,np.arange(len(wall)),1.,1.)
 assert track.debug['left_wall_valid'] and geometry_valid(result),track.debug
 assert np.ptp(wall[:,0])<track.c.min_wall_span_m
 assert track.debug['forward_observed_length']>=track.c.min_forward_observed_m


def test_normal_corridor_without_common_wall_x_reaches_both_consumers():
 from team_01_race.lidar_filter import CameraWallFilter
 x=np.linspace(.2,.95,100);wall=np.column_stack((x,1.2+2*x))
 track=LidarTrack();track.note_width(2.4,0.,1.)
 d=track.update(wall,np.arange(len(wall)),1.,1.)
 assert geometry_valid(d)
 assert d['left'][-1,0]<d['right'][0,0]
 f=CameraWallFilter(.242)
 assert f.update_camera(d['path'],d['left'],d['right'],d['confidence'],d['observed'],1.,1.)
 ready,obstacles,_=f.classify(wall,np.arange(len(wall)),1.,1.)
 assert ready and not obstacles
 e=DecisionEngine(vehicle_width=.242,wheelbase=.34,require_official=False)
 e.controller_ready=True;e.set_light('LIGHTS_OUT')
 for i in range(3):
  stamp=1+i*.04;e.camera(d['path'],d['left'],d['right'],d['confidence'],d['observed'],i,stamp,stamp=stamp);e.lidar([],stamp,stamp=stamp)
 assert e.path_confirmed and e.step(stamp)['enabled']


def test_folding_outer_wall_supports_sharp_turn_with_recent_width():
 from team_01_race.lidar_filter import CameraWallFilter
 theta=np.linspace(.04,1.65,280);radius=1.7;half=1.2
 outer=np.column_stack(((radius+half)*np.sin(theta),-radius+(radius+half)*np.cos(theta)))
 # The scan order is far-to-near on the left wall.
 outer=outer[::-1];track=LidarTrack();track.note_width(2*half,0.,1.)
 d=track.update(outer,np.arange(len(outer)),1.,1.)
 assert track.debug['parametric_wall_candidates']>0
 assert geometry_valid(d),track.debug
 assert np.any(np.diff(d['left'][:,0])<=0) or np.any(np.diff(d['right'][:,0])<=0)
 assert inside_corridor(d['path'],d['left'],d['right'],.271)
 f=CameraWallFilter(.242)
 assert f.update_camera(d['path'],d['left'],d['right'],d['confidence'],d['observed'],1.,1.)
 ready,objects,_=f.classify(outer,np.arange(len(outer)),1.,1.)
 assert ready and not objects


def test_occluded_wall_uses_long_measured_opposite_wall_with_own_width():
 p,l,r=corridor();track=LidarTrack()
 q,idx=laser_walls(p,l,r);both=track.update(q,idx,1.,1.)
 assert geometry_valid(both) and all(both['observed'])
 short=r[r[:,0]<=1.6]
 q,idx=laser_walls(p,l,short)
 fallback=track.update(q,idx,1.05,1.05)
 assert geometry_valid(fallback),track.debug
 assert fallback['observed']==(True,False) and fallback['confidence']<.7
 assert fallback['path'][0,0]<.5
 assert 4.<fallback['path'][-1,0]<=l[-1,0]+1e-8
 assert np.max(abs(fallback['path'][:,1]))<.02
 # No Camera width/reference supplied, and a lone wall cannot renew width age.
 left=q[np.isclose(q[:,1],1.2)]
 assert track.update(left,np.arange(len(left)),3.2,3.2) is None


@pytest.mark.parametrize('mirror',[False,True])
def test_recorded_right_pass_near_and_far_wall_support_are_joined_without_gap(mirror):
 import json
 from team_01_race.lidar_filter import scan_points
 f=json.loads((Path(__file__).parent/'fixtures/right_pass_wall_occlusion.json').read_text())
 if mirror:f['ranges']=f['ranges'][::-1]
 p,l,r=corridor();track=LidarTrack();q,idx=laser_walls(p,l,r)
 assert geometry_valid(track.update(q,idx,f['stamp']-.1,f['stamp']-.1))
 valid,q,idx=scan_points(f['ranges'],f['angle_min'],f['angle_increment'],f['range_min'],f['range_max'],np.eye(3),np.array([.195,0.,.02]),5.,20)
 d=track.update(q,idx,f['stamp'],f['stamp'])
 assert valid and geometry_valid(d),track.debug
 # Metric wall merging can now supply the same observed prefix directly;
 # verify coverage and conservative single-wall authority below.
 assert d['path'][0,0]<.6 and d['path'][-1,0]>4.5
 assert not all(d['observed']) and d['confidence']<.7
 assert d['path'][-1,0]<=q[:,0].max()


def test_single_wall_candidates_with_a_gap_or_disagreement_cannot_be_joined():
 p,l,r=corridor();track=LidarTrack();track.width=2.4
 a=(2.,1,[p[:20],l[:20],r[:20]])
 b=(2.,-1,[p[30:],l[30:],r[30:]])
 assert track._join_wall_prefixes([a,b]) is None
 shifted=[v+[0,.3] for v in (p,l,r)]
 assert track._join_wall_prefixes([a,(5.,-1,shifted)]) is None


@pytest.mark.parametrize('slope',[0.,.07])
def test_short_associated_opposite_wall_validates_width_without_extending_single_wall_age(slope):
 p,l,r=corridor();p[:,1]+=slope*p[:,0];l[:,1]+=slope*l[:,0];r[:,1]+=slope*r[:,0];track=LidarTrack();q,idx=laser_walls(p,l,r)
 assert geometry_valid(track.update(q,idx,1.,1.))
 short=l[l[:,0]<=.7+1e-8];q,idx=laser_walls(p,short,r)
 for i in range(1,61):
  now=1.+i*.05;d=track.update(q,idx,now,now)
  assert geometry_valid(d),track.debug
  assert now-track.width_time<.1
  assert d['path'][0,0]<.5 and d['path'][-1,0]>4.
 # A truly single wall cannot renew the original two-wall observation.
 x=np.arange(.1,5.,.025);q=np.column_stack((x,np.full(len(x),-1.2)));idx=np.arange(len(x))
 assert track.update(q,idx,now+2.1,now+2.1) is None


def test_short_compact_center_object_is_not_a_trusted_wall():
 p,l,r=corridor();track=LidarTrack();q,idx=laser_walls(p,l,r)
 assert geometry_valid(track.update(q,idx,1.,1.))
 x=np.arange(.2,.9,.025);car=np.column_stack((x,np.full(len(x),.45)))
 d=track.update(car,np.arange(len(car)),1.05,1.05)
 assert d is None and not track.wall_indices


def test_metric_contiguous_wall_seam_does_not_drop_near_prefix():
 p,l,r=corridor();q,idx=laser_walls(p,l,r)
 n=len(q)//2;idx=idx.copy();idx[40:n]+=4;idx[n:]+=10
 track=LidarTrack();d=track.update(q,idx,1.,1.)
 assert geometry_valid(d) and all(d['observed'])
 assert d['path'][0,0]<.5


def monaco_contour():
 import json
 from team_01_race.lidar_filter import scan_points
 data=json.loads((Path(__file__).parent/'fixtures/monaco_connected_contour.json').read_text())
 valid,points,indices=scan_points(data['ranges'],data['angle_min'],data['angle_increment'],
  data['range_min'],data['range_max'],np.eye(3),data['translation'],5.,20)
 assert valid
 return points,indices


def test_connected_sag_contour_recovers_two_current_walls_after_width_expiry():
 points,indices=monaco_contour();track=LidarTrack()
 # A cold start and an expired prior both need current, independently measured walls.
 for now in (1.,4.):
  d=track.update(points,indices,now,now)
  assert geometry_valid(d) and d['observed']==(True,True),track.debug
  assert track.debug['connected_contour_splits']==1
  assert d['confidence']>.85 and now-track.width_time<1e-6
  assert 1.5<d['track_width']<1.9 and d['path'][-1,0]>3.5
  assert abs(d['path'][0,1])<.1


def test_connected_contour_does_not_label_the_transverse_returns_as_wall():
 points,indices=monaco_contour();track=LidarTrack()
 assert geometry_valid(track.update(points,indices,1.,1.))
 transverse=(points[:,0]>4.2)&(abs(points[:,1])<.3)
 assert transverse.sum()>10
 assert not set(indices[transverse])&set(track.wall_indices)


def closed_corridor(distance):
 right=np.column_stack((np.linspace(.2,distance,100),np.full(100,-.8)))
 front=np.column_stack((np.full(100,distance),np.linspace(-.8,.8,100)))
 left=np.column_stack((np.linspace(distance,.2,100),np.full(100,.8)))
 points=np.r_[right,front[1:],left[1:]]
 return points,np.arange(len(points))


def test_connected_physical_barrier_limits_observed_path_and_remains_unlabelled():
 points,indices=closed_corridor(2.);track=LidarTrack()
 d=track.update(points,indices,1.,1.)
 assert geometry_valid(d),track.debug
 assert d['path'][-1,0]<=2.+1e-6
 front=(points[:,0]==2.)&(abs(points[:,1])<.3)
 assert not set(indices[front])&set(track.wall_indices)


def test_near_closed_barrier_cannot_produce_a_usable_forward_path():
 points,indices=closed_corridor(.45);track=LidarTrack()
 assert track.update(points,indices,1.,1.) is None


def seam_scan(key):
 import json
 from team_01_race.lidar_filter import scan_points
 fixture=json.loads((Path(__file__).parent/'fixtures/monaco_isolated_seam_returns.json').read_text())
 data=fixture[key]
 valid,points,indices=scan_points(data['ranges'],data['angle_min'],data['angle_increment'],
  data['range_min'],data['range_max'],np.eye(3),[.195,0,.02],5.,20)
 assert valid
 return fixture,points,indices


def test_isolated_return_cannot_split_a_metric_continuous_wall_prefix():
 from team_01_race.lidar_filter import groups
 _,points,indices=seam_scan('528');track=LidarTrack();before=points.copy()
 d=track.update(points,indices,1.,1.)
 assert geometry_valid(d),track.debug
 assert track.debug['left_wall_valid'] and track.debug['right_wall_valid']
 assert track.debug['right_wall_points']>400
 assert track.debug['intervening_unfitted_wall_groups']>=1
 noise=np.concatenate([indices[g] for g in groups(points,indices,.18) if len(g)<3])
 assert len(noise)>0 and not set(noise)&set(track.wall_indices)
 np.testing.assert_array_equal(points,before)


def test_measured_short_inside_wall_revalidates_width_after_seam_join():
 fixture,points,indices=seam_scan('528');track=LidarTrack()
 assert geometry_valid(track.update(points,indices,1.,1.))
 motion=fixture['previous_to_current']
 ref=track.previous@np.array(motion['rotation'])+np.array(motion['translation'])
 track.previous=ref[ref[:,0]>0]
 assert valid_path(track.previous)
 _,points,indices=seam_scan('530')
 d=track.update(points,indices,1.05,1.05)
 assert geometry_valid(d) and all(d['observed']),track.debug
 assert track.width_time==1.05 and d['path'][-1,0]>1.
 # Expired memory is still insufficient to bootstrap from only this short wall.
 cold=LidarTrack()
 assert cold.update(points,indices,4.,4.) is None
