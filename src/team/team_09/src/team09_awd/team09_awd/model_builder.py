"""Generate the submitted SDF and bridge from the SAME configuration as planning."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .config import G, SETTINGS, topic, frame
from .kinematics import WHEELS


def fingerprint(cfg=None):
    return hashlib.sha256(json.dumps(cfg or SETTINGS,sort_keys=True,separators=(",",":")).encode()).hexdigest()


def add(parent, tag, text=None, **attrs):
    node=ET.SubElement(parent,tag,attrs)
    if text is not None:
        node.text=" ".join(format(float(x),".12g") for x in text) if isinstance(text,(list,tuple)) else str(text)
    return node


def inertia(link, mass, diag, xyz=None):
    node=add(link,"inertial")
    if xyz is not None: add(node,"pose",[*xyz,0,0,0])
    add(node,"mass",mass); node=add(node,"inertia")
    for key,value in zip(("ixx","iyy","izz"),diag): add(node,key,value)
    for key in ("ixy","ixz","iyz"): add(node,key,0)


def shape(link, name, kind, dimensions, pose, color):
    for tag in ("collision","visual"):
        obj=add(link,tag,name=name+"_"+tag);add(obj,"pose",pose)
        geom=add(add(obj,"geometry"),kind)
        if kind=="box": add(geom,"size",dimensions)
        else:
            add(geom,"radius",dimensions[0]);add(geom,"length",dimensions[1])
        if tag=="visual":
            mat=add(obj,"material");add(mat,"ambient",color);add(mat,"diffuse",color)
        else:
            ode=add(add(add(obj,"surface"),"friction"),"ode")
            add(ode,"mu",SETTINGS["geometry"]["tyre_friction"]);add(ode,"mu2",SETTINGS["geometry"]["tyre_friction"])


def contact(link, collisions, name):
    sensor=add(link,"sensor",name="contact_"+name,type="contact")
    add(sensor,"always_on","true");add(sensor,"update_rate",50)
    add(sensor,"topic",topic("contacts/"+name))
    obj=add(sensor,"contact")
    for collision in collisions: add(obj,"collision",collision)


def dimensions():
    """Neutral collision AABB and conservative full-steer width, in base frame."""
    g=SETTINGS["geometry"];sx,sy,sz=g["body_size_m"]
    lo=[-sx/2,-max(sy/2,G.track/2+g["wheel_width_m"]/2),min(g["body_z_m"]-sz/2,G.ground_z)]
    hi=[sx/2,-lo[1],max(g["body_z_m"]+sz/2,g["wheel_z_m"]+G.radius)]
    lo[0]=min(lo[0],-G.wheelbase/2-G.radius);hi[0]=max(hi[0],G.wheelbase/2+G.radius)
    for box in g["sensor_housings"]:
        for i in range(3):
            lo[i]=min(lo[i],box["xyz"][i]-box["size"][i]/2)
            hi[i]=max(hi[i],box["xyz"][i]+box["size"][i]/2)
    return {"min_m":lo,"max_m":hi,"size_m":[hi[i]-lo[i] for i in range(3)],
            "centre_m":[(hi[i]+lo[i])/2 for i in range(3)],
            "full_steer_width_m":2*(G.half_width-g["footprint_padding_m"])}


def validate_dimensions():
    d=dimensions()
    if any(abs(v)>1e-9 for v in d["centre_m"]):
        raise ValueError("base_link is not at collision AABB centre: "+str(d))
    sizes=d["size_m"]
    if sizes[0]>.55 or max(sizes[1],d["full_steer_width_m"])>.25 or sizes[2]>.18:
        raise ValueError("Vehicle exceeds v24 envelope: "+str(d))
    return d


def build_sdf():
    validate_dimensions();g=SETTINGS["geometry"];s=SETTINGS["sensors"]
    sdf=ET.Element("sdf",version="1.9");model=add(sdf,"model",name=SETTINGS["entry_id"])
    # Upstream practice worlds supply rendering sensors, but no IMU/Contact
    # systems. These standard systems can be loaded on the spawned model.
    # Disable this option if the operator supplies these systems globally.
    if s["load_imu_contact_systems"]:
        add(model,"plugin",filename="gz-sim-imu-system",name="gz::sim::systems::Imu")
        add(model,"plugin",filename="gz-sim-contact-system",name="gz::sim::systems::Contact")
    add(model,"self_collide","false");body=add(model,"link",name="base_link")
    mass=g["body_mass_kg"];sx,sy,sz=g["body_size_m"]
    inertia(body,mass,[mass*(sy*sy+sz*sz)/12,mass*(sx*sx+sz*sz)/12,mass*(sx*sx+sy*sy)/12],[0,0,g["body_z_m"]])
    shape(body,"body","box",g["body_size_m"],[0,0,g["body_z_m"],0,0,0],[.05,.3,.85,1])
    for box in g["sensor_housings"]:
        shape(body,box["name"],"box",box["size"],[*box["xyz"],0,0,0],[.15,.15,.18,1])
    contact(body,["body_collision"]+[b["name"]+"_collision" for b in g["sensor_housings"]],"base_link")
    imu=add(body,"sensor",name="body_imu",type="imu")
    add(imu,"always_on","true");add(imu,"update_rate",s["imu_hz"]);add(imu,"topic",topic("imu"));add(imu,"gz_frame_id",frame("base_link"));add(imu,"imu")
    lidar=add(body,"sensor",name="lidar",type="gpu_lidar")
    add(lidar,"pose",s["lidar_pose"]);add(lidar,"topic",topic("lidar"));add(lidar,"gz_frame_id",frame("lidar"))
    add(lidar,"always_on","true");add(lidar,"update_rate",s["lidar_hz"])
    l=add(lidar,"lidar");scan=add(l,"scan");h=add(scan,"horizontal")
    for k,v in {"samples":s["lidar_horizontal_samples"],"resolution":1,"min_angle":-math.radians(s["lidar_fov_deg"])/2,"max_angle":math.radians(s["lidar_fov_deg"])/2}.items():add(h,k,v)
    v=add(scan,"vertical");n=1 if s["lidar_mode"]=="2d" else s["lidar_vertical_samples_3d"]
    angle=0 if n==1 else math.radians(s["lidar_vertical_fov_deg"])/2
    for k,x in {"samples":n,"resolution":1,"min_angle":-angle,"max_angle":angle}.items():add(v,k,x)
    r=add(l,"range")
    for k,x in {"min":s["lidar_min_m"],"max":s["lidar_max_m"],"resolution":.09}.items():add(r,k,x)
    for prefix,kind,key in (("camera","rgbd_camera","camera_pose"),("start_camera","camera","start_camera_pose")):
        sensor=add(body,"sensor",name=prefix,type=kind)
        add(sensor,"pose",s[key]);add(sensor,"topic",topic(prefix if prefix=="camera" else prefix+"/image"))
        add(sensor,"always_on","true");add(sensor,"update_rate",s["camera_hz"]);add(sensor,"gz_frame_id",frame(prefix+"_optical"))
        camera=add(sensor,"camera");add(camera,"camera_info_topic",topic(prefix+"/camera_info"))
        add(camera,"horizontal_fov",s["camera_hfov_rad"] if prefix=="camera" else s["start_camera_hfov_rad"])
        im=add(camera,"image")
        for k,x in {"width":s["camera_width"],"height":s["camera_height"],"format":"R8G8B8"}.items():add(im,k,x)
        clip=add(camera,"clip");add(clip,"near",s["camera_near_m"]);add(clip,"far",s["camera_far_m"])
    for name in WHEELS:
        front=name.startswith("front");left=name.endswith("left")
        xyz=[(1 if front else -1)*G.wheelbase/2,(1 if left else -1)*G.track/2,g["wheel_z_m"]]
        if front:
            kn=add(model,"link",name=name+"_knuckle");add(kn,"pose",[*xyz,0,0,0]);inertia(kn,g["knuckle_mass_kg"],[6.66666667e-7]*3)
            j=add(model,"joint",name=name+"_steer",type="revolute")
            add(j,"parent","base_link");add(j,"child",name+"_knuckle")
            axis=add(j,"axis");add(axis,"xyz",[0,0,1]);lim=add(axis,"limit")
            for k,x in {"lower":-G.max_wheel_angle,"upper":G.max_wheel_angle,"effort":g["steer_effort_nm"],"velocity":g["steer_joint_rate_radps"]}.items():add(lim,k,x)
            add(add(axis,"dynamics"),"damping",.1)
        wheel=add(model,"link",name=name+"_wheel");add(wheel,"pose",[*xyz,0,0,0])
        m=g["wheel_mass_kg"];r=G.radius;w=g["wheel_width_m"];iy=m*r*r/2;ix=m*(3*r*r+w*w)/12
        inertia(wheel,m,[ix,iy,ix]);shape(wheel,name+"_wheel","cylinder",[r,w],[0,0,0,math.pi/2,0,0],[.04,.04,.05,1])
        contact(wheel,[name+"_wheel_collision"],name+"_wheel")
        j=add(model,"joint",name=name+"_drive",type="revolute")
        add(j,"parent",name+"_knuckle" if front else "base_link");add(j,"child",name+"_wheel")
        axis=add(j,"axis");add(axis,"xyz",[0,1,0]);lim=add(axis,"limit")
        add(lim,"effort",g["drive_effort_nm"]);add(lim,"velocity",g["drive_joint_rate_radps"])
        plugin=add(model,"plugin",filename="gz-sim-joint-controller-system",name="gz::sim::systems::JointController")
        add(plugin,"joint_name",name+"_drive");add(plugin,"topic",topic("drive/"+name));add(plugin,"initial_velocity",0)
        add(plugin,"use_force_commands","false")
    plugin=add(model,"plugin",filename="gz-sim-ackermann-steering-system",name="gz::sim::systems::AckermannSteering")
    for k,v in {"steering_only":"true","left_steering_joint":"front_left_steer","right_steering_joint":"front_right_steer",
                "wheel_base":G.wheelbase,"wheel_separation":G.track,"kingpin_width":G.track,
                "wheel_radius":G.radius,"steering_limit":G.max_centre_angle,"steer_p_gain":g["steer_p_gain"],"topic":topic("drive/steer_angle")}.items():add(plugin,k,v)
    plugin=add(model,"plugin",filename="gz-sim-joint-state-publisher-system",name="gz::sim::systems::JointStatePublisher")
    add(plugin,"topic",topic("joint_states"));add(plugin,"update_rate",SETTINGS["actuator"]["joint_state_hz"])
    for j in [x+"_drive" for x in WHEELS]+["front_left_steer","front_right_steer"]:add(plugin,"joint_name",j)
    ET.indent(sdf)
    return '<?xml version="1.0"?>\n<!-- generated config sha256: '+fingerprint()+' -->\n'+ET.tostring(sdf,encoding="unicode")+"\n"


def bridge_config():
    rows=[]
    def row(suffix,ros,gz,direction="GZ_TO_ROS"):
        rows.append(dict(ros_topic_name=topic(suffix),gz_topic_name=topic(suffix),ros_type_name=ros,gz_type_name=gz,direction=direction,publisher_queue=1,subscriber_queue=1))
    row("lidar/points","sensor_msgs/msg/PointCloud2","gz.msgs.PointCloudPacked")
    row("imu","sensor_msgs/msg/Imu","gz.msgs.IMU")
    row("joint_states","sensor_msgs/msg/JointState","gz.msgs.Model")
    for suffix in ("camera/image","camera/depth_image","start_camera/image"):row(suffix,"sensor_msgs/msg/Image","gz.msgs.Image")
    for suffix in ("camera/camera_info","start_camera/camera_info"):row(suffix,"sensor_msgs/msg/CameraInfo","gz.msgs.CameraInfo")
    for name in ("base_link",)+tuple(x+"_wheel" for x in WHEELS):row("contacts/"+name,"ros_gz_interfaces/msg/Contacts","gz.msgs.Contacts")
    for suffix in ("drive/steer_angle",)+tuple("drive/"+x for x in WHEELS):row(suffix,"std_msgs/msg/Float64","gz.msgs.Double","ROS_TO_GZ")
    return rows


def write_generated(package_root):
    root=Path(package_root)
    (root/"models/car").mkdir(parents=True,exist_ok=True)
    (root/"models/car/model.sdf").write_text(build_sdf())
    # JSON is a YAML 1.2 subset, accepted by yaml-cpp used by ros_gz_bridge.
    (root/"config/bridge.yaml").write_text(json.dumps(bridge_config(),indent=2)+"\n")
    (root/"config/generated.json").write_text(json.dumps({"config_sha256":fingerprint(),"dimensions":dimensions()},indent=2)+"\n")
