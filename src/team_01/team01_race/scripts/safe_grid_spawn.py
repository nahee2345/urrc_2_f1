#!/usr/bin/env python3
"""CI-only spawn clearance correction; NOT imported by autonomous controller.

The official number-one painted grid offset on some worlds leaves less than
half a car width to the wall. Read only the verifier reference to position an
unobstructed *test vehicle*. Keep the original grid heading and long position.
Record the exact correction for honest reporting.
"""
import argparse, json, math, sys
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from team01_race.lap_validation import LapVerifier, load_reference


def adjusted_grid(points, widths, x, y, yaw, *, min_clearance_m=.33):
    v=LapVerifier(points,widths)
    p=np.array((x,y), dtype=float)
    delta=p-points
    t=np.clip(np.sum(delta*v.seg,axis=1)/v.seg_sq,0,1)
    nearest=points+t[:,None]*v.seg
    errors=np.linalg.norm(nearest-p,axis=1)
    index=int(np.argmin(errors))
    norm=np.array((-v.seg[index,1],v.seg[index,0]))/v.seg_len[index]
    signed=float(np.dot(p-nearest[index],norm))
    width=float(widths[index]*(1-t[index])+widths[(index+1)%len(widths)]*t[index])
    # Need a physical body-width + steering reserve; never exceed a small
    # correction that could relocate the vehicle into a different grid slot.
    allowed=max(.05,width/2.-float(min_clearance_m))
    dest=float(np.clip(signed,-allowed,allowed))
    shift=dest-signed
    if abs(shift)>.45:
        raise ValueError('Grid spawn too close to wall; >45 cm correction is not allowed')
    target=p+shift*norm
    result={"x":round(float(target[0]),8),"y":round(float(target[1]),8),
            "yaw":round(float(yaw),8),"original_x":float(x),"original_y":float(y),
            "shift_m":round(float(abs(shift)),5),"original_lateral_m":round(signed,5),
            "new_lateral_m":round(dest,5),"road_width_m":round(width,5),
            "source":"CI_ONLY_OFFICIAL_REFERENCE_NOT_VISIBLE_TO_CONTROLLER"}
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--reference',required=True)
    p.add_argument('--report',required=True)
    p.add_argument('--x',type=float,default=.37867234638)
    p.add_argument('--y',type=float,default=-2.05603539766)
    p.add_argument('--yaw',type=float,default=1.45738441607)
    p.add_argument('--min-clearance',type=float,default=.33)
    a=p.parse_args()
    result=adjusted_grid(*load_reference(a.reference),a.x,a.y,a.yaw,min_clearance_m=a.min_clearance)
    Path(a.report).parent.mkdir(parents=True,exist_ok=True)
    Path(a.report).write_text(json.dumps(result,indent=2)+'\n')
    print(f'{result["x"]} {result["y"]} {result["yaw"]}')

if __name__=='__main__':main()
