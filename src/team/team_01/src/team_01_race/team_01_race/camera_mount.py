"""One calibrated base_link mount for SDF generation, TF and perception."""
from dataclasses import dataclass,asdict
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml

@dataclass
class CameraMount:
    x_m: float=.23
    y_m: float=0.
    z_m: float=.08
    roll_deg: float=0.
    pitch_deg: float=12.
    yaw_deg: float=0.
    use_camera_mast: bool=False
    mast_height_m: float=0.
    camera_forward_offset_m: float=.23
    camera_pitch_deg: float=12.
    allow_oversize_for_validation: bool=False

    def pose(self):
        d=asdict(self)
        if any(not math.isfinite(v) for v in d.values()):raise ValueError('nonfinite camera mount')
        if not 0<=self.mast_height_m<=.12:raise ValueError('mast lift must be in [0,.12] m')
        return np.array([self.camera_forward_offset_m if self.use_camera_mast else self.x_m,self.y_m,
            self.z_m+(self.mast_height_m if self.use_camera_mast else 0),math.radians(self.roll_deg),
            math.radians(self.camera_pitch_deg if self.use_camera_mast else self.pitch_deg),math.radians(self.yaw_deg)])

def load_mount(path):
    d=yaml.safe_load(open(path))
    if '/**' in d:d=d['/**']['ros__parameters']
    return CameraMount(**d['camera_mount'])

def rotation(r,p,y):
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def set_text(node,key,value):
    item=node.find(key)
    if item is None:item=ET.SubElement(node,key)
    item.text=str(value)

def model_envelope(tree):
    """All visual/collision protrusions including the full steering sweep."""
    lo=np.full(3,np.inf);hi=-lo
    model=tree.getroot().find('model')
    limit=float(model.findtext("joint[@name='front_left_steering_joint']/axis/limit/upper"))
    for link in model.findall('link'):
        p=np.fromstring(link.findtext('pose','0 0 0 0 0 0'),sep=' ')
        angles=np.linspace(-limit,limit,101) if link.get('name','').startswith('front_') and link.get('name','').endswith('_wheel') else [0.]
        for item in [*link.findall('visual'),*link.findall('collision')]:
            q=np.fromstring(item.findtext('pose','0 0 0 0 0 0'),sep=' ')
            for angle in angles:
                R=rotation(0,0,angle)@rotation(*p[3:]);center=p[:3]+R@q[:3];R=R@rotation(*q[3:])
                box=item.findtext('geometry/box/size');cylinder=item.find('geometry/cylinder')
                if box is not None:extent=np.abs(R)@np.fromstring(box,sep=' ')/2
                elif cylinder is not None:
                    axis=R[:,2];radius=float(cylinder.findtext('radius'));length=float(cylinder.findtext('length'))
                    extent=abs(axis)*length/2+radius*np.sqrt(np.maximum(0,1-axis*axis))
                else:raise ValueError('unsupported geometry in envelope check')
                lo=np.minimum(lo,center-extent);hi=np.maximum(hi,center+extent)
    return dict(minimum_m=lo.tolist(),maximum_m=hi.tolist(),dimensions_m=(hi-lo).tolist(),official_limits_m=[.55,.25,.18],compliant=bool(np.all(hi-lo<=np.array([.55,.25,.18])+1e-9)))

def configured_tree(filename,mount):
    tree=ET.parse(filename);m=tree.getroot().find('model');camera=m.find("link[@name='camera_link']")
    old=np.array([float(v) for v in camera.findtext('pose').split()]);ch=np.array([float(v) for v in m.findtext("link[@name='base_link']/pose").split()])
    p=mount.pose();p[:3]+=ch[:3]
    camera.find('pose').text=' '.join(map(str,p))
    # Keep existing total mass and model-frame COM when relocating the sensor.
    mass=float(camera.findtext('inertial/mass'));old_inertial=np.fromstring(camera.findtext('inertial/pose','0 0 0 0 0 0'),sep=' ');target=old[:3]+rotation(*old[3:])@old_inertial[:3];newmass=mass
    existing=m.find("link[@name='camera_mast_link']")
    if existing is not None:
        mm=float(existing.findtext('inertial/mass'))
        center=np.fromstring(existing.findtext('pose'),sep=' ')[:3]
        target=(mass*target+mm*center)/(mass+mm);mass+=mm;newmass=mass
        m.remove(existing)
        joint=m.find("joint[@name='camera_mast_fixed_joint']")
        if joint is not None:m.remove(joint)
        m.find("joint[@name='camera_fixed_joint']/parent").text='base_link'
    original_center=target.copy()
    if mount.use_camera_mast:
        anchor=ch[:3]+np.array([p[0]-ch[0],p[1]-ch[1],.03]);end=p[:3]-np.array([0,0,.016]);length=end[2]-anchor[2]
        if length<=0:raise ValueError('mast top must clear base_link anchor')
        center=(anchor+end)/2;mm=1e-6;newmass=mass-mm
        rod=ET.SubElement(m,'link',name='camera_mast_link');ET.SubElement(rod,'pose').text=' '.join(map(str,[*center,0,0,0]))
        inert=ET.SubElement(rod,'inertial');ET.SubElement(inert,'mass').text=str(mm)
        inertia=ET.SubElement(inert,'inertia')
        for key,value in [('ixx',1e-10),('iyy',1e-10),('izz',1e-10)]:ET.SubElement(inertia,key).text=str(value)
        vis=ET.SubElement(rod,'visual',name='camera_mast');geo=ET.SubElement(vis,'geometry');box=ET.SubElement(geo,'box');ET.SubElement(box,'size').text=f'.008 .008 {length}'
        mat=ET.SubElement(vis,'material');ET.SubElement(mat,'ambient').text='.1 .1 .1 1';ET.SubElement(mat,'diffuse').text='.1 .1 .1 1'
        joint=ET.SubElement(m,'joint',name='camera_mast_fixed_joint',type='fixed');ET.SubElement(joint,'parent').text='base_link';ET.SubElement(joint,'child').text='camera_mast_link'
        m.find("joint[@name='camera_fixed_joint']/parent").text='camera_mast_link'
        target=(mass*original_center-mm*center)/newmass
    camera.find('inertial/mass').text=str(newmass)
    offset=rotation(*p[3:]).T@(target-p[:3]);set_text(camera.find('inertial'),'pose',' '.join(map(str,[*offset,0,0,0])))
    # Official limit includes sensor protrusions. Original housing is already
    # oversize; never silently certify an even taller research mast.
    envelope=model_envelope(tree)
    if not envelope['compliant'] and not mount.allow_oversize_for_validation:
        raise ValueError(f"camera mast dimensions {envelope['dimensions_m']} exceed official [.55,.25,.18]m; validation-only override required")
    return tree
