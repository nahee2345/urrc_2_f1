"""Generate SDF 1.9 dynamic 4-wheel cars, without world modification.

Gazebo Harmonic controllers use joint motor *forces* and steering PID forces.
All dimensions, including full steering sweep, must remain within the race envelope.
The model origin is at axle height: spawn at road_z + wheel_radius.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

STEER_LIMIT_DEG = 35.0
LIMITS = (0.55, 0.25, 0.18)
SAFE_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass(frozen=True)
class Vehicle:
    name: str = "urrc_stage1_car"
    drive: str = "rwd"
    body_length: float = 0.46
    body_width: float = 0.16
    body_height: float = 0.06
    body_center_z: float = 0.025
    wheelbase: float = 0.34
    wheel_track: float = 0.156
    wheel_radius: float = 0.04
    wheel_width: float = 0.024
    body_mass: float = 1.40
    wheel_mass: float = 0.07
    knuckle_mass: float = 0.03
    wheel_friction: float = 1.0
    motor_max_torque: float = 0.08
    steer_max_torque: float = 1.0
    max_steer_deg: float = STEER_LIMIT_DEG

    @classmethod
    def from_file(cls, filename: Path | str):
        data = json.loads(Path(filename).read_text(encoding="utf-8"))
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown configuration keys: {sorted(unknown)}")
        return cls(**data)

    @property
    def mass(self):
        return self.body_mass + 4 * self.wheel_mass + 2 * self.knuckle_mass

    def envelope(self):
        """Conservative geometric envelope, including front wheel steering sweep.

        No sensor meshes are included in stage 1; stage 3 must recheck the total.
        """
        half_x = self.body_length / 2
        half_y = self.body_width / 2
        half_z_lower = self.body_center_z - self.body_height / 2
        half_z_upper = self.body_center_z + self.body_height / 2
        # Wheels are cylinders aligned with Y, then rotated about Z in steering.
        for n in range(0, 36):
            a = math.radians(self.max_steer_deg * n / 35)
            for angle in (a, -a):
                ex = self.wheel_radius * abs(math.cos(angle)) + self.wheel_width / 2 * abs(math.sin(angle))
                ey = self.wheel_radius * abs(math.sin(angle)) + self.wheel_width / 2 * abs(math.cos(angle))
                half_x = max(half_x, self.wheelbase / 2 + ex)
                half_y = max(half_y, self.wheel_track / 2 + ey)
        # rear wheels do not steer
        half_y = max(half_y, self.wheel_track / 2 + self.wheel_width / 2)
        # wheel bottoms at -radius, tops at +radius relative to model origin
        bottom = min(-self.wheel_radius, half_z_lower)
        top = max(self.wheel_radius, half_z_upper)
        return (2 * half_x, 2 * half_y, top - bottom)

    def validate(self):
        if not SAFE_NAME.fullmatch(self.name):
            raise ValueError("name must be a unique alphanumeric / underscore Gazebo model name")
        if self.drive not in {"fwd", "rwd", "awd"}:
            raise ValueError("drive must be fwd, rwd, or awd")
        values = [getattr(self, x) for x in self.__dataclass_fields__ if x not in {"name", "drive"}]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in values):
            # body_center_z is deliberately positive in this prototype
            raise ValueError("all numeric dimensions, masses and dynamics values must be positive and finite")
        if self.max_steer_deg > STEER_LIMIT_DEG:
            raise ValueError("front steering limit exceeds 35 degrees")
        if self.wheelbase >= self.body_length:
            raise ValueError("wheelbase must be shorter than the body")
        if self.wheel_track < self.body_width / 2:
            raise ValueError("wheel track would bring wheel centers too close together")
        if self.body_center_z - self.body_height / 2 <= -self.wheel_radius + 0.008:
            raise ValueError("chassis may strike ground; insufficient ground clearance")
        envelope = self.envelope()
        if any(actual > maximum + 1e-9 for actual, maximum in zip(envelope, LIMITS)):
            raise ValueError(f"vehicle full steering envelope {envelope!r} exceeds race maximum {LIMITS!r}")


def fmt(*numbers):
    return " ".join(f"{num:.9g}" for num in numbers)


def sub(parent, tag, value=None, **attrib):
    el = ET.SubElement(parent, tag, attrib)
    if value is not None:
        el.text = str(value)
    return el


def inertial(link, mass, ix, iy, iz):
    part = sub(link, "inertial")
    sub(part, "mass", fmt(mass))
    inertia = sub(part, "inertia")
    for k, v in (("ixx", ix), ("iyy", iy), ("izz", iz), ("ixy", 0), ("ixz", 0), ("iyz", 0)):
        sub(inertia, k, fmt(v))


def box_inertia(m, x, y, z):
    return (m * (y*y + z*z) / 12, m * (x*x + z*z) / 12, m * (x*x + y*y) / 12)


def cylinder_inertia_y(m, radius, width):
    radial = m * (3*radius*radius + width*width) / 12
    return radial, m * radius*radius/2, radial


def box_geometry(parent, dims):
    geometry = sub(parent, "geometry")
    sub(sub(geometry, "box"), "size", fmt(*dims))


def wheel_geometry(parent, radius, width):
    # Cylinder's local Z must be turned onto axle Y.
    sub(parent, "pose", fmt(0, 0, 0, math.pi/2, 0, 0))
    geom = sub(parent, "geometry")
    cyl = sub(geom, "cylinder")
    sub(cyl, "radius", fmt(radius))
    sub(cyl, "length", fmt(width))


def make_joint(model, name, parent, child, axis, *, bounds=None, effort=None, velocity=None, damping=0.02):
    j = sub(model, "joint", type="revolute", name=name)
    # The joint zero pose is coincident with its child link frame.
    sub(j, "pose", fmt(0, 0, 0, 0, 0, 0), relative_to=child)
    sub(j, "parent", parent)
    sub(j, "child", child)
    a = sub(j, "axis")
    sub(a, "xyz", fmt(*axis), expressed_in="__model__")
    limit = sub(a, "limit")
    if bounds is not None:
        sub(limit, "lower", fmt(bounds[0]))
        sub(limit, "upper", fmt(bounds[1]))
    if effort is not None:
        sub(limit, "effort", fmt(effort))
    if velocity is not None:
        sub(limit, "velocity", fmt(velocity))
    dynamics = sub(a, "dynamics")
    sub(dynamics, "damping", fmt(damping))
    sub(dynamics, "friction", fmt(0))
    return j


def make_plugin(model, name, filename, joint_name, topic, parameters):
    p = sub(model, "plugin", name=name, filename=filename)
    sub(p, "joint_name", joint_name)
    sub(p, "topic", topic)
    for key, value in parameters.items():
        sub(p, key, str(value).lower() if isinstance(value, bool) else fmt(value))


def generate(v: Vehicle):
    v.validate()
    sdf = ET.Element("sdf", version="1.9")
    model = sub(sdf, "model", name=v.name, canonical_link="base_link")
    sub(model, "static", "false")
    sub(model, "self_collide", "false")
    body = sub(model, "link", name="base_link")
    sub(body, "pose", fmt(0,0,v.body_center_z,0,0,0))
    inertial(body, v.body_mass, *box_inertia(v.body_mass,v.body_length,v.body_width,v.body_height))
    for kind in ("visual", "collision"):
        part=sub(body,kind,name=f"body_{kind}")
        box_geometry(part,(v.body_length,v.body_width,v.body_height))
        if kind=="visual":
            # 무광에 가까운 검정 차체 (Gazebo 조명 아래 회색으로 보일 수 있음).
            material=sub(part,"material")
            sub(material,"ambient","0.015 0.015 0.018 1")
            sub(material,"diffuse","0.028 0.028 0.033 1")
            sub(material,"specular","0.045 0.045 0.045 1")
    for wheel in WHEELS:
        front = wheel.startswith("front")
        left = wheel.endswith("left")
        x = v.wheelbase / 2 * (1 if front else -1)
        y = v.wheel_track / 2 * (1 if left else -1)
        parent = "base_link"
        if front:
            kn = sub(model,"link",name=f"{wheel}_knuckle")
            sub(kn,"pose",fmt(x,y,0,0,0,0))
            dims=(0.022,0.020,0.018)
            inertial(kn,v.knuckle_mass,*box_inertia(v.knuckle_mass,*dims))
            box_geometry(sub(kn,"visual",name=f"{wheel}_knuckle_visual"),dims)
            # No extra collision on the overlapped steering hub; the outer wheel has collision.
            steer=f"{wheel}_steer_joint"
            angle=math.radians(v.max_steer_deg)
            make_joint(model,steer,"base_link",f"{wheel}_knuckle",(0,0,1),bounds=(-angle,angle),effort=v.steer_max_torque,velocity=math.pi/2)
            side="left" if left else "right"
            make_plugin(model,"gz::sim::systems::JointPositionController","gz-sim-joint-position-controller-system",steer,
                        f"/model/{v.name}/steer/{side}",
                        {"p_gain":8.0,"i_gain":0.0,"d_gain":0.25,"cmd_max":v.steer_max_torque,"cmd_min":-v.steer_max_torque})
            parent=f"{wheel}_knuckle"
        wl=sub(model,"link",name=f"{wheel}_wheel")
        sub(wl,"pose",fmt(x,y,0,0,0,0))
        inertial(wl,v.wheel_mass,*cylinder_inertia_y(v.wheel_mass,v.wheel_radius,v.wheel_width))
        for kind in ("visual","collision"):
            node=sub(wl,kind,name=f"{wheel}_{kind}")
            wheel_geometry(node,v.wheel_radius,v.wheel_width)
            if kind=="collision":
                friction=sub(sub(sub(node,"surface"),"friction"),"ode")
                sub(friction,"mu",fmt(v.wheel_friction))
                sub(friction,"mu2",fmt(v.wheel_friction))
        spin=f"{wheel}_wheel_joint"
        make_joint(model,spin,parent,f"{wheel}_wheel",(0,1,0),effort=v.motor_max_torque * 2,velocity=200,damping=0.005)
        driven=(v.drive=="awd" or (v.drive=="fwd" and front) or (v.drive=="rwd" and not front))
        if driven:
            make_plugin(model,"gz::sim::systems::JointController","gz-sim-joint-controller-system",spin,
                        f"/model/{v.name}/drive/{wheel}",
                        {"use_force_commands":True,"p_gain":0.2,"i_gain":0.02,
                         "cmd_max":v.motor_max_torque,"cmd_min":-v.motor_max_torque,
                         "i_max":v.motor_max_torque,"i_min":-v.motor_max_torque})
    # Publish joint state for independent inspection, no need for GT/pose in a controller.
    js=sub(model,"plugin",name="gz::sim::systems::JointStatePublisher",filename="gz-sim-joint-state-publisher-system")
    for wheel in WHEELS:
        sub(js,"joint_name",f"{wheel}_wheel_joint")
        if wheel.startswith("front"):
            sub(js,"joint_name",f"{wheel}_steer_joint")
    ET.indent(sdf, space="  ")
    return ET.tostring(sdf,encoding="unicode",xml_declaration=True)


def manifest(v:Vehicle):
    v.validate()
    return {"name":v.name,"drive":v.drive,"mass_kg":v.mass,
            "size_m_conservative":dict(zip(("length","width","height"),v.envelope())),
            "wheel_radius_m":v.wheel_radius,"wheelbase_m":v.wheelbase,
            "spawn_z_offset_m":v.wheel_radius,"steer_limit_deg":v.max_steer_deg,
            "drive_topics":[f"/model/{v.name}/drive/{w}" for w in WHEELS if v.drive=="awd" or (v.drive=="fwd" and w.startswith("front")) or (v.drive=="rwd" and w.startswith("rear"))],
            "steer_topics":[f"/model/{v.name}/steer/{side}" for side in ("left","right")],
            "notes":"Stage 1 geometry; sensor installations not included. Recheck full envelope at Stage 3."}
