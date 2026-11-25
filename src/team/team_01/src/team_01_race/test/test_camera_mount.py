import math,sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.camera_mount import CameraMount,configured_tree,rotation,load_mount
from team_01_race.vision import VisionConfig,CameraVision
ROOT=Path(__file__).resolve().parents[1];MODEL=ROOT/'models/team_01_car/model.sdf'

def test_original_mount_and_config_match_model():
 m=load_mount(ROOT/'config/camera_mount.yaml');t=configured_tree(MODEL,m)
 assert np.allclose(np.fromstring(t.findtext("model/link[@name='camera_link']/pose"),sep=' '),[.23,0,.125,0,math.radians(12.),0])
 assert t.find("model/link[@name='camera_mast_link']") is None

def mass_com(tree):
 mass=0.;moment=np.zeros(3)
 for link in tree.findall('model/link'):
  p=np.fromstring(link.findtext('pose','0 0 0 0 0 0'),sep=' ');q=np.fromstring(link.findtext('inertial/pose','0 0 0 0 0 0'),sep=' ')
  m=float(link.findtext('inertial/mass','0'));mass+=m;moment+=m*(p[:3]+rotation(*p[3:])@q[:3])
 return mass,moment/mass

def test_mast_preserves_mass_and_com_and_is_reversible():
 a=configured_tree(MODEL,CameraMount());b=configured_tree(MODEL,CameraMount(use_camera_mast=True,allow_oversize_for_validation=True))
 assert b.findtext("model/joint[@name='camera_fixed_joint']/parent")=='camera_mast_link'
 am,ac=mass_com(a);bm,bc=mass_com(b)
 assert am==pytest.approx(bm,abs=1e-12)
 assert np.allclose(ac,bc,atol=1e-12)
 assert b.find("model/link[@name='camera_mast_link']/collision") is None
 assert configured_tree(MODEL,CameraMount()).find("model/link[@name='camera_mast_link']") is None

def test_official_limit_rejects_oversize_mast():
 with pytest.raises(ValueError,match='official'):configured_tree(MODEL,CameraMount(use_camera_mast=True,mast_height_m=.08))

def test_projection_matches_independent_full_extrinsic_transform():
 c=VisionConfig(camera_x=.13,camera_y=.07,camera_height=.26,camera_roll=.08,camera_pitch=.15,camera_yaw=-.1)
 v=CameraVision(c);p=np.array([[1.,.3],[3.,-.4],[9.,0.]])
 k=np.array([[185.,0,320],[0,185.,240],[0,0,1]])
 R=rotation(c.camera_roll,c.camera_pitch,c.camera_yaw)
 world=np.column_stack((p,np.zeros(len(p))))
 optical=np.array([[0,-1,0],[0,0,-1],[1,0,0]])@R.T@(world-np.array([c.camera_x,c.camera_y,c.camera_height])).T
 image=k@optical;expected=(image[:2]/image[2]).T
 assert np.allclose(v.project(p,k),expected)
 # Optical principal point is not the vehicle centreline with y offset.
 assert abs(v.project(np.array([[3.,0.]]),k)[0,0]-320)>1


def test_offset_camera_image_recovers_vehicle_centred_road():
 import cv2
 c=VisionConfig(camera_x=.05,camera_y=.09,camera_height=.26,camera_roll=.04,camera_pitch=.15,camera_yaw=.03)
 v=CameraVision(c);k=np.array([[185.,0,320],[0,185.,240],[0,0,1]])
 image=np.full((480,640,3),(100,170,100),np.uint8)
 road=v.project([[.3,1.2],[.3,-1.2],[30,-1.2],[30,1.2]],k).astype(np.int32)
 cv2.fillPoly(image,[road],(65,65,65))
 for y in (-1.2,1.2):cv2.polylines(image,[v.project([[.3,y],[30,y]],k).astype(np.int32)],False,(255,255,255),6)
 result=v.process(image,k,np.zeros(5),0.)
 assert result['path'] is not None
 assert abs(np.median(result['path'][:,1]))<.04


def test_legal_support_toggle_retains_sensor_pose_and_size():
 from team_01_race.camera_mount import model_envelope
 a=configured_tree(MODEL,CameraMount(use_camera_mast=False))
 b=configured_tree(MODEL,CameraMount(use_camera_mast=True))
 assert a.findtext("model/link[@name='camera_link']/pose")==b.findtext("model/link[@name='camera_link']/pose")
 assert model_envelope(a)['compliant'] and model_envelope(b)['compliant']
 am,ac=mass_com(a);bm,bc=mass_com(b)
 assert am==pytest.approx(bm,abs=1e-12) and np.allclose(ac,bc,atol=1e-12)


def test_generated_mast_model_can_be_reconfigured_without_duplicate_links(tmp_path):
 m=CameraMount(use_camera_mast=True);a=configured_tree(MODEL,m);source=tmp_path/'mounted.sdf';a.write(source)
 b=configured_tree(source,m)
 assert len(b.findall("model/link[@name='camera_mast_link']"))==1
 am,ac=mass_com(a);bm,bc=mass_com(b)
 assert am==pytest.approx(bm,abs=1e-12) and np.allclose(ac,bc,atol=1e-12)
 c=configured_tree(source,CameraMount(use_camera_mast=False))
 assert c.find("model/link[@name='camera_mast_link']") is None
