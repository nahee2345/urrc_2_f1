import sys,json
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from team_01_race.camera_bundle import encode_bundle,decode_bundle
from team_01_race.ros_support import path_array

def packet():
 x=np.arange(.1,5.01,.1);p=np.column_stack((x,.1*x*x));return dict(path=p,left=p+[0,1.2],right=p-[0,1.2],confidence=1.,observed=(True,True),light='LIGHTS_OUT',reason='both_camera_walls')

def test_atomic_bundle_keeps_one_stamp_and_all_arrays():
 h=Header(frame_id='team_01/camera_optical_frame',stamp=Time(sec=1,nanosec=120000000));r=packet();parts=decode_bundle(encode_bundle(r,h),1.14)
 assert len(parts)==4
 for name,key in [('center_path','path'),('left_boundary','left'),('right_boundary','right')]:
  assert np.array_equal(path_array(parts[name]),r[key])
  assert parts[name].header.stamp==h.stamp and parts[name].header.frame_id=='team_01/base_link'

def test_empty_geometry_is_delivered_as_a_real_path_loss_frame():
 r=packet();r.update(path=None,left=None,right=None,confidence=0.,observed=(False,False))
 p=decode_bundle(encode_bundle(r,Header(stamp=Time(sec=1))),1.01)
 assert path_array(p['center_path']).shape==(0,2)
 assert p['status'].status[0].values[0].value=='0.0'

@pytest.mark.parametrize('change',[{'frame_id':'world'},{'confidence':float('nan')},{'nanosec':1000000000},{'observed':[True]},{'traffic_light':'bad'}])
def test_bad_atomic_header_or_metadata_cannot_renew_geometry(change):
 d=json.loads(encode_bundle(packet(),Header(stamp=Time(sec=1))));d.update(change)
 with pytest.raises((ValueError,TypeError)):decode_bundle(json.dumps(d),1.01)

def test_stale_and_future_bundle_rejected():
 data=encode_bundle(packet(),Header(stamp=Time(sec=1)))
 for now in (.8,1.3):
  with pytest.raises(ValueError):decode_bundle(data,now)


def lidar_frame():
 from team_01_race.camera_bundle import encode_lidar_frame
 r=packet();r['path_source']='LIDAR_PATH'
 return encode_lidar_frame(r,Header(stamp=Time(sec=1,nanosec=50000000)),
     np.array([[1.,1.2],[1.5,1.2],[2.,0.]]),np.array([10,11,12]),[10,11],True)


def test_lidar_scan_wall_mask_and_obstacles_preserve_capture_stamp_end_to_end():
 from team_01_race.camera_bundle import decode_lidar_frame,encode_lidar_observation,decode_lidar_observation
 from team_01_race.navigation import Obstacle
 frame=lidar_frame();parts,points,indices,walls,valid=decode_lidar_frame(frame,1.10)
 assert valid and walls=={10,11} and indices.tolist()==[10,11,12]
 assert np.array_equal(points,[[1.,1.2],[1.5,1.2],[2.,0.]])
 data=encode_lidar_observation(frame,[Obstacle('car',2.,0.,.2,.3,1.,.7)],True)
 received,obstacles,valid=decode_lidar_observation(data,1.20)
 assert valid and obstacles[0]['closing_speed']==.7
 for name in received:assert received[name].header.stamp==parts[name].header.stamp
 # Processing and another transport hop cannot re-date an old observation.
 for now in (1.31,.9):
  with pytest.raises(ValueError):decode_lidar_observation(data,now)


@pytest.mark.parametrize('change',[{'wall_indices':[99]},{'indices':[10,10,12]},
 {'points':[[1.,float('nan')],[1.5,1.2],[2.,0.]]},{'valid':1},
 {'points':[[1.],[2.],[3.],[4.],[5.],[6.]]}])
def test_misaligned_or_corrupt_scan_cannot_supply_synchronized_clear_space(change):
 from team_01_race.camera_bundle import decode_lidar_frame
 data=json.loads(lidar_frame());data['lidar_frame'].update(change)
 with pytest.raises((ValueError,TypeError)):decode_lidar_frame(json.dumps(data),1.1)


def test_unusable_scan_is_distinct_from_valid_empty_observation():
 from team_01_race.camera_bundle import encode_lidar_observation,decode_lidar_observation
 for valid in (False,True):
  _,obstacles,received=decode_lidar_observation(encode_lidar_observation(lidar_frame(),[],valid),1.1)
  assert received is valid and not obstacles
