"""Slope holding torque through the existing force-mode wheel velocity PID.

With P-only JointController, torque = P*(omega_command-omega_measured).
Adding tau_hold/P to its target adds tau_hold to the existing damping torque.
No second force publisher may fight the wheel controller.
"""
import math
import xml.etree.ElementTree as ET
from .ramp import RampDetector


def brake_model(filename):
    model=ET.parse(filename).getroot().find('model')
    mass=sum(float(link.findtext('inertial/mass','0')) for link in model.findall('link'))
    wheels=[]
    for side in ('left','right'):
        joint='rear_'+side+'_wheel_joint'
        plugins=[p for p in model.findall('plugin') if p.findtext('joint_name')==joint]
        if len(plugins)!=1:raise ValueError('one rear wheel controller required')
        p=plugins[0]
        if (p.findtext('use_force_commands')!='true' or
            float(p.findtext('i_gain','0'))!=0 or float(p.findtext('d_gain','0'))!=0):
            raise ValueError('slope hold requires force-mode P-only wheel controller')
        gain=float(p.findtext('p_gain'));limit=min(float(p.findtext('cmd_max')),-float(p.findtext('cmd_min')))
        wheels.append((gain,limit))
    if not all(math.isfinite(v) and v>0 for v in (mass,*(v for w in wheels for v in w))):
        raise ValueError('invalid brake model')
    return mass,wheels


class RampBrake:
    def __init__(self,mass,radius,wheels,detector=None):
        if len(wheels)!=2 or not all(math.isfinite(v) and v>0 for v in (mass,radius,*(v for w in wheels for v in w))):
            raise ValueError('invalid brake configuration')
        self.mass,self.radius,self.wheels=mass,radius,wheels
        self.imu=detector or RampDetector()
        self.hold_pitch=None

    def update(self,quaternion,stamp,now,**kwargs):
        return self.imu.update(quaternion,stamp,now,**kwargs)

    def command(self,now,*,stop_requested):
        fresh=self.imu.fresh(now)
        if not stop_requested:
            self.hold_pitch=None
        elif fresh:
            if self.imu.active:self.hold_pitch=self.imu.filtered
            elif self.imu.exit_count==0:self.hold_pitch=None
        # If IMU expires while holding, retain the last measured compensation.
        # Stale IMU cannot establish a new measured compensation.
        active=stop_requested and self.hold_pitch is not None
        torque=-self.mass*9.81*self.radius*math.sin(math.radians(self.hold_pitch))/2 if active else 0.
        torques=[max(-limit,min(limit,torque)) for gain,limit in self.wheels]
        return dict(active=active,imu_fresh=fresh,holding_last_pitch=bool(active and not fresh),
                    pitch_deg=self.hold_pitch,required_torque_nm_per_wheel=torque,
                    commanded_torque_nm_per_wheel=torques,
                    saturated=any(abs(torque)>limit for gain,limit in self.wheels),
                    velocity_bias_radps=[t/gain for t,(gain,limit) in zip(torques,self.wheels)])
