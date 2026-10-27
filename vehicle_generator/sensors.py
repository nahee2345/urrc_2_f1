"""Stage 2: configurable REAL Gazebo sensors on the Stage 1 physical chassis.

Physical housings participate in mass, inertia, dimensions and collisions.
One common, dedicated start-light camera is isolated from driving sensor selection.
Driving sensors only observe their own Gazebo measurements (no ground truth).
"""
from __future__ import annotations
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .model import Vehicle, LIMITS, fmt, generate, sub, box_inertia

# Centers are in MODEL coordinates (+X forward, +Y left, +Z up).
# Lens / emitter is moved beyond its housing to avoid scanning through itself.
# All measurements depend on Gazebo Sensors+Ogre2, NOT mock/random data.
SENSOR_SPECS = {
    "start_camera": {"type":"camera","center":(0.235,0.000,0.065),
                     "size":(0.018,0.018,0.020),"mass":0.022,"rate":15,
                     "pitch":-0.55, "width":160,"height":120},
    "rgb": {"type":"camera","center":(0.235,0.034,0.065),
            "size":(0.018,0.022,0.020),"mass":0.030,"rate":15,
            "pitch":0.0,"width":160,"height":120},
    "depth": {"type":"depth_camera","center":(0.235,-0.034,0.065),
              "size":(0.018,0.022,0.020),"mass":0.040,"rate":10,
              "pitch":0.0,"width":160,"height":120},
    "stereo_left": {"type":"camera","center":(0.235,0.066,0.065),
                    "size":(0.018,0.016,0.020),"mass":0.025,"rate":15,
                    "pitch":0.0,"width":160,"height":120},
    "stereo_right": {"type":"camera","center":(0.235,-0.066,0.065),
                     "size":(0.018,0.016,0.020),"mass":0.025,"rate":15,
                     "pitch":0.0,"width":160,"height":120},
    "lidar2d": {"type":"gpu_lidar","center":(-0.125,0.000,0.072),
                "size":(0.034,0.036,0.032),"mass":0.050,"rate":10,
                "beams":1,"h_samples":360},
    "lidar3d": {"type":"gpu_lidar","center":(-0.040,0.000,0.075),
                "size":(0.054,0.054,0.040),"mass":0.085,"rate":10,
                "beams":16,"h_samples":360},
}
DRIVING = set(SENSOR_SPECS) - {"start_camera"}
ALIASES = {"stereo":("stereo_left","stereo_right")}
KNOWN = set(DRIVING) | set(ALIASES)
DEFAULT_PRESETS = Path(__file__).resolve().parents[1] / "config" / "stage2_sensor_presets.json"


def sensors_for(driving):
    if not isinstance(driving,list) or len(driving)==0:
        raise ValueError("driving sensors must be a nonempty list")
    names=["start_camera"]
    for entry in driving:
        if entry not in KNOWN:
            raise ValueError(f"unsupported driving sensor: {entry!r}")
        names.extend(ALIASES.get(entry,(entry,)))
    if len(names)!=len(set(names)):
        raise ValueError("duplicate sensor or stereo alias overlap")
    return names


def bounding(v:Vehicle, names):
    v.validate()
    length,width,height=v.envelope()
    xlo,xhi=-length/2,length/2
    ylo,yhi=-width/2,width/2
    zlo=-v.wheel_radius
    zhi=max(v.wheel_radius,v.body_center_z+v.body_height/2)
    for name in names:
        spec=SENSOR_SPECS[name]
        c=spec["center"];half=tuple(d/2 for d in spec["size"])
        xlo=min(xlo,c[0]-half[0]); xhi=max(xhi,c[0]+half[0])
        ylo=min(ylo,c[1]-half[1]); yhi=max(yhi,c[1]+half[1])
        zlo=min(zlo,c[2]-half[2]); zhi=max(zhi,c[2]+half[2])
        # Optical/ray origin has no hardware exterior, but include in conservative bound.
        optical=optical_center(name)
        xlo=min(xlo,optical[0]);xhi=max(xhi,optical[0])
        ylo=min(ylo,optical[1]);yhi=max(yhi,optical[1])
        zlo=min(zlo,optical[2]);zhi=max(zhi,optical[2])
    result=(xhi-xlo,yhi-ylo,zhi-zlo)
    if any(a>b+1e-9 for a,b in zip(result,LIMITS)):
        raise ValueError(f"sensor-equipped car envelope {result} exceeds {LIMITS}")
    return result


def optical_center(name):
    s=SENSOR_SPECS[name]
    x,y,z=s["center"]
    if s["type"]=="gpu_lidar":
        return x,y,z+s["size"][2]/2+0.003
    return x+s["size"][0]/2+0.003,y,z


def add_box(link, kind, name, center, size, body_z):
    elem=sub(link,kind,name=name)
    sub(elem,"pose",fmt(center[0],center[1],center[2]-body_z,0,0,0))
    sub(sub(sub(elem,"geometry"),"box"),"size",fmt(*size))


def _set_composite_inertia(link,v:Vehicle,names):
    """Proper composite COM and full inertia tensor incl. off-diagonal products.

    Link frame is original chassis centroid (0,0,body_center_z in model frame).
    Each mass element is a uniform box, with inertia via parallel axes.
    """
    components=[(v.body_mass,(0.,0.,0.),(v.body_length,v.body_width,v.body_height))]
    for name in names:
        spec=SENSOR_SPECS[name]
        p=spec["center"]
        components.append((spec["mass"],(p[0],p[1],p[2]-v.body_center_z),spec["size"]))
    total=sum(m for m,_,_ in components)
    com=tuple(sum(m*p[i] for m,p,_ in components)/total for i in range(3))
    Ixx=Iyy=Izz=Ixy=Ixz=Iyz=0.
    for m,p,d in components:
        dx,dy,dz=(p[i]-com[i] for i in range(3))
        ix,iy,iz=box_inertia(m,*d)
        Ixx+=ix+m*(dy*dy+dz*dz)
        Iyy+=iy+m*(dx*dx+dz*dz)
        Izz+=iz+m*(dx*dx+dy*dy)
        Ixy-=m*dx*dy
        Ixz-=m*dx*dz
        Iyz-=m*dy*dz
    inertial=link.find("inertial")
    sub(inertial,"pose",fmt(*com,0,0,0))
    inertial.find("mass").text=fmt(total)
    for tag,value in (("ixx",Ixx),("iyy",Iyy),("izz",Izz),
                      ("ixy",Ixy),("ixz",Ixz),("iyz",Iyz)):
        inertial.find("inertia/"+tag).text=fmt(value)
    return total,com


def _camera(sensor,spec):
    cam=sub(sensor,"camera")
    sub(cam,"horizontal_fov","1.20")
    img=sub(cam,"image")
    sub(img,"width",str(spec["width"]))
    sub(img,"height",str(spec["height"]))
    sub(img,"format","R_FLOAT32" if spec["type"]=="depth_camera" else "R8G8B8")
    clip=sub(cam,"clip")
    sub(clip,"near","0.05")
    sub(clip,"far","30.0")


def _lidar(sensor,spec):
    ray=sub(sensor,"ray")
    scan=sub(ray,"scan")
    horizontal=sub(scan,"horizontal")
    for k,v in (("samples",spec["h_samples"]),("resolution",1),
                ("min_angle",-math.pi),("max_angle",math.pi)):
        sub(horizontal,k,fmt(v))
    vertical=sub(scan,"vertical")
    n=spec["beams"]
    for k,v in (("samples",n),("resolution",1),
                ("min_angle",-0.25 if n>1 else 0),("max_angle",0.25 if n>1 else 0)):
        sub(vertical,k,fmt(v))
    distance=sub(ray,"range")
    for k,v in (("min",0.08),("max",18.0),("resolution",0.01)):
        sub(distance,k,fmt(v))


def build(v:Vehicle,driving):
    names=sensors_for(driving)
    size=bounding(v,names)
    root=ET.fromstring(generate(v))
    model=root.find("model")
    link=model.find("link[@name='base_link']")
    total_mass,com=_set_composite_inertia(link,v,names)
    topics={}
    for name in names:
        spec=SENSOR_SPECS[name]
        center=spec["center"]
        add_box(link,"visual",f"{name}_housing_visual",center,spec["size"],v.body_center_z)
        add_box(link,"collision",f"{name}_housing_collision",center,spec["size"],v.body_center_z)
        sensor=sub(link,"sensor",name=name,type=spec["type"])
        ox,oy,oz=optical_center(name)
        sub(sensor,"pose",fmt(ox,oy,oz-v.body_center_z,0,spec.get("pitch",0),0))
        sub(sensor,"always_on","true")
        sub(sensor,"update_rate",str(spec["rate"]))
        sub(sensor,"visualize","false")
        topic=f"/model/{v.name}/sensors/{name}"
        sub(sensor,"topic",topic)
        if spec["type"]=="gpu_lidar":
            _lidar(sensor,spec)
            topics[name]={"gazebo":topic,"gz_type":"gz.msgs.LaserScan",
                           "ros_type":"sensor_msgs/msg/LaserScan",
                           "vertical_beams":spec["beams"],
                           "pointcloud_topic_candidate":topic+"/points"}
        else:
            _camera(sensor,spec)
            topics[name]={"gazebo":topic,"gz_type":"gz.msgs.Image",
                           "ros_type":"sensor_msgs/msg/Image",
                           "width":spec["width"],"height":spec["height"]}
    ET.indent(root,space="  ")
    report={
        "vehicle":v.name,"drive":v.drive,"driving_sensors":list(driving),
        "start_only_sensor":"start_camera",
        "sensor_count":len(names),"total_mass_kg":round(total_mass + 4*v.wheel_mass + 2*v.knuckle_mass,6),
        "sensor_mass_kg":round(sum(SENSOR_SPECS[n]["mass"] for n in names),6),
        "chassis_com_in_link_m":[round(c,8) for c in com],
        "envelope_m":dict(zip(("length","width","height"),[round(x,8) for x in size])),
        "topics":topics,
        "limitations":"Stage 2: sensor topics and physics; ROS bridging and light perception algorithm are separate validation gates.",
    }
    return ET.tostring(root,encoding="unicode",xml_declaration=True),report


def load_presets(path:Path|str=DEFAULT_PRESETS):
    data=json.loads(Path(path).read_text(encoding="utf-8"))
    if sorted(data)!=[f"AI-{n:02d}" for n in range(1,11)]:
        raise ValueError("exactly AI-01 through AI-10 required")
    for ai,c in data.items():
        if set(c)!={"drive","driving"}:
            raise ValueError(f"invalid config keys: {ai}")
        if c["drive"] not in ("fwd","rwd","awd"):
            raise ValueError(f"invalid drivetrain: {ai}")
        sensors_for(c["driving"])
    return data
