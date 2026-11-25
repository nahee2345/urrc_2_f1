#!/usr/bin/env python3
"""Primitive SDF bounds at neutral and full independent steering sweep.
Exact support functions for boxes/cylinders. Continuous sweep enclosed by
sampling at <=0.1 degree and a Lipschitz outward margin for moving geometry.
Unsupported pose frames, geometry or model nesting fail explicitly.
"""
from pathlib import Path
import xml.etree.ElementTree as E
import math,argparse,json

def mul(a,b):return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def rot(r,p,y):
 c,s=math.cos,math.sin
 return mul(mul([[c(y),-s(y),0],[s(y),c(y),0],[0,0,1]],[[c(p),0,s(p)],[0,1,0],[-s(p),0,c(p)]]),[[1,0,0],[0,c(r),-s(r)],[0,s(r),c(r)]])
def vec(a,b):return [sum(a[i][j]*b[j] for j in range(3)) for i in range(3)]
def pose(e):
 p=e.find('pose')
 if p is not None and p.attrib:raise ValueError('relative pose frames unsupported')
 return list(map(float,(p.text if p is not None else '0 0 0 0 0 0').split()))
def bounds(path):
 model=E.parse(path).getroot().find('model');assert model is not None
 assert not model.findall('model'), 'nested models unsupported'
 result=[]
 for sweep in [False,True]:
  for kind in ['visual','collision']:
   lo=[math.inf]*3;hi=[-math.inf]*3;count=0
   for l in model.findall('link'):
    lp=pose(l);lr=rot(*lp[3:]);moving='front_' in l.get('name') and ('wheel' in l.get('name') or 'knuckle' in l.get('name'))
    for g in l.findall(kind):
     count+=1;gp=pose(g);gr=rot(*gp[3:]);geom=g.find('geometry');box=geom.find('box');cyl=geom.find('cylinder')
     if box is not None:half=[float(t)/2 for t in box.findtext('size').split()];radius=math.sqrt(sum(t*t for t in half))
     elif cyl is not None:r=float(cyl.findtext('radius'));h=float(cyl.findtext('length'))/2;radius=math.hypot(r,h)
     else:raise ValueError('unsupported geometry: '+g.get('name'))
     margin=(radius+math.sqrt(sum(t*t for t in gp[:3])))*math.radians(.1)/2 if moving and sweep else 0
     for degree in ([-35+i*.1 for i in range(701)] if moving and sweep else [0]):
      lr2=mul(rot(0,0,math.radians(degree)),lr);a=mul(lr2,gr);p=[x+y for x,y in zip(lp[:3],vec(lr2,gp[:3]))]
      if box is not None:ext=[sum(abs(a[i][j])*half[j] for j in range(3)) for i in range(3)]
      else:ext=[r*math.sqrt(max(0,1-a[i][2]**2))+h*abs(a[i][2]) for i in range(3)]
      for i in range(3):lo[i]=min(lo[i],p[i]-ext[i]-(margin if i<2 else 0));hi[i]=max(hi[i],p[i]+ext[i]+(margin if i<2 else 0))
   size=[hi[i]-lo[i] for i in range(3)];assert count
   result.append(dict(state='sweep ±35deg' if sweep else 'neutral',kind=kind,min=lo,max=hi,size=size,
                      target=all(v<=lim+1e-9 for v,lim in zip(size,[.52,.24,.165])),legal=all(v<=lim+1e-9 for v,lim in zip(size,[.55,.25,.18]))))
 for l in model.findall('link'):
  vs=l.findall('visual');cs=l.findall('collision');assert len(vs)==len(cs)
  for v,c in zip(vs,cs):
   assert pose(v)==pose(c) and [(e.tag,(e.text or '').strip()) for e in v.find('geometry').iter()]==[(e.tag,(e.text or '').strip()) for e in c.find('geometry').iter()], 'visual/collision mismatch'
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('model',nargs='?',type=Path,default=Path(__file__).resolve().parents[1]/'models/team04_car/model.sdf');p.add_argument('--json',type=Path);args=p.parse_args()
 result=bounds(args.model)
 print('state          geometry    length(m) width(m) height(m) target legal')
 for row in result:print(f"{row['state']:14} {row['kind']:10} "+' '.join(f'{x:.6f}' for x in row['size'])+f" {row['target']} {row['legal']}")
 if args.json:args.json.write_text(json.dumps(result,indent=2)+'\n')
 assert all(r['legal'] and r['target'] for r in result),'dimension limit exceeded'
