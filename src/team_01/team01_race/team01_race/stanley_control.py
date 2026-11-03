"""Kinematic Stanley tracker evaluated on the sensed gradual local reference.

One steering law is used for prediction and the selected first command.
Local road safety remains enforced separately by the collision envelope.
"""
import math
import numpy as np
from team01_race.rl_tuner import action as learned_action


def stanley_steer(reference,x,y,yaw,speed,wheelbase,max_steer,cfg):
    ref=np.asarray(reference,dtype=float)
    if ref.ndim!=2 or len(ref)<4 or ref.shape[1]!=2:
        raise ValueError('reference path unavailable')
    px,py=ref[:,0],ref[:,1]
    speed=max(0.,float(speed))
    # A short lookahead prevents reacting to sensor-level center jitter.
    look=float(np.clip(1.25+.15*speed,1.25,2.7))
    q=min(float(px[-1])-.3,max(float(px[0])+.3,float(x)+look))
    h=min(.35,max(.08,float(px[-1]-px[0])*.05))
    ya=float(np.interp(max(px[0],q-h),px,py))
    yb=float(np.interp(min(px[-1],q+h),px,py))
    slope=(yb-ya)/max(.05,min(px[-1],q+h)-max(px[0],q-h))
    heading=math.atan(slope)
    eheading=math.atan2(math.sin(heading-yaw),math.cos(heading-yaw))
    nx=min(float(px[-1]),max(float(px[0]),float(x)+.85))
    cross=float(np.interp(nx,px,py))-float(y)-math.sin(yaw)*(nx-float(x))
    # Experimental toy-simulator policy; explicit opt-in only.
    gains=(learned_action(str(cfg.get('learned_gains_file','')),
                          cross,eheading,speed)
           if cfg.get('enable_learned_gains',False) else None)
    if gains is not None:
        chosen_look,_,_=gains
        q=min(float(px[-1])-.3,max(float(px[0])+.3,float(x)+chosen_look))
        ya=float(np.interp(max(px[0],q-h),px,py))
        yb=float(np.interp(min(px[-1],q+h),px,py))
        slope=(yb-ya)/max(.05,min(px[-1],q+h)-max(px[0],q-h))
        eheading=math.atan2(math.sin(math.atan(slope)-yaw),
                            math.cos(math.atan(slope)-yaw))
    # Curvature feedforward is limited independently of feedback.
    d0=float(np.interp(max(px[0],q-.5),px,py))
    d1=float(np.interp(q,px,py))
    d2=float(np.interp(min(px[-1],q+.5),px,py))
    curvature=float(np.clip((d2-2*d1+d0),-.25,.25))
    feedforward=math.atan(float(wheelbase)*curvature)
    kheading=float(gains[1]) if gains is not None else float(cfg.get('stanley_heading_gain',.85))
    kcross=float(gains[2]) if gains is not None else float(cfg.get('stanley_cross_gain',1.0))
    soft=float(cfg.get('stanley_speed_softening_mps',1.5))
    raw=(feedforward+kheading*eheading+
         math.atan2(kcross*cross,speed+soft))
    return float(np.clip(raw,-max_steer,max_steer))
