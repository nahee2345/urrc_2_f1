"""Diagnostic interpretation separated from steering control."""
import math

def attitude(x,y,z,w):
 n=math.sqrt(x*x+y*y+z*z+w*w)
 if not math.isfinite(n) or n<1e-9:return math.pi,0.
 x,y,z,w=(a/n for a in (x,y,z,w))
 return math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1.,min(1.,2*(w*y-z*x))))

def depth_state(status,age,pose_ok,fused):
 if not 0<=age<.2:return 'STALE_OR_MISSING'
 if status!='ROUTE_VALID':return status
 if not pose_ok:return 'NO_DEPTH_POSE'
 return 'FUSED' if fused else 'VALID_NOT_FUSED'
