import math
from pathlib import Path
from team09_awd.diagnostics import attitude,depth_state

def test_tipped_attitude():
 roll,pitch=attitude(math.sin(math.pi/4),0,0,math.cos(math.pi/4))
 assert abs(roll-math.pi/2)<1e-9 and abs(pitch)<1e-9
 assert attitude(0,0,0,0)[0]==math.pi

def test_depth_receiving_is_not_fusing():
 assert depth_state('NO_VALID_ROAD',.1,True,False)=='NO_VALID_ROAD'
 assert depth_state('ROUTE_VALID',.1,True,False)=='VALID_NOT_FUSED'
 assert depth_state('ROUTE_VALID',.3,True,True)=='STALE_OR_MISSING'
 assert depth_state('ROUTE_VALID',.1,False,False)=='NO_DEPTH_POSE'
 assert depth_state('ROUTE_VALID',.1,True,True)=='FUSED'
