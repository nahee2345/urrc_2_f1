#!/usr/bin/env python3
"""Plot recorded failure geometry and same-input safe trajectory; validation only."""
import json,math,argparse
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from team10_race.footprint_trajectory import plan_footprint_trajectory
from team10_race.wall_tracker import WallObservation,_binned
from team10_race.lidar_processing import scan_to_points,continuous_segments
p=argparse.ArgumentParser();p.add_argument('--replay',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
rows=[json.loads(x) for x in Path(a.replay).read_text().splitlines()]
fig,axes=plt.subplots(2,2,figsize=(13,12));summary=[]
for i,at in enumerate([208.56,211.036]):
 r=min(rows,key=lambda x:abs(x['stamp']-at));q=r['scan'];s=SimpleNamespace(**q);s.ranges=[v if v is not None else math.inf for v in s.ranges]
 pts=scan_to_points(s.ranges,s.angle_min,s.angle_increment,s.range_min,s.range_max,270,.25,24);seg=continuous_segments(pts,.3,2,8);g=r['geometry'];cur=g.get('command',{}).get('steering_angle',0)
 trajectory=plan_footprint_trajectory(s,seg,WallObservation(None,None,1.72),cur,1.)
 for j in range(2):
  ax=axes[i,j]
  for k,chain in enumerate(seg):
   bins=_binned([v.point for v in chain if -.1<=v.point.x<=16],.2)
   rejected=len(bins)<8
   ax.plot([v.point.x for v in chain],[v.point.y for v in chain],'.',ms=2,label=f'wall support {k}: {len(chain)} returns / {len(bins)} x bins'+(' REJECT <8' if rejected else ''))
   if rejected and bins:ax.scatter([v.x for v in bins],[v.y for v in bins],s=18,marker='x',c='darkorange')
  ax.add_patch(Polygon([[-.34,-.116],[.196,-.116],[.196,.116],[-.34,.116]],facecolor='gray',alpha=.5,label='F1 full envelope, laser frame'))
  for side in ['left','right']:
   wall=g.get('observation',{}).get(side)
   if wall:
    xs=np.linspace(wall['x_min'],wall['x_max'],80);co=wall['coefficients'];ax.plot(xs,co[0]+co[1]*xs+co[2]*xs*xs,'m-',label=f'accepted {side} wall fit')
  poly=g.get('polynomial',{}).get('points',[])
  if poly:ax.plot([v['x'] for v in poly],[v['y'] for v in poly],'m--',label='polynomial centre candidate')
  if j==0:
   path=g.get('speed_path',[])
   if path:ax.plot([0]+[v['x'] for v in path],[0]+[v['y'] for v in path],'r.-',label='before: speed collision path')
   if r['corridor_hits']:
    h=r['corridor_hits'][0];o=h['obstacle'];v=h['projection'];ax.plot(o['x'],o['y'],'r*',ms=14,label='blocking return');ax.plot([o['x'],v['x']],[o['y'],v['y']],'r:');ax.scatter(v['x'],v['y'],facecolors='none',edgecolors='red',s=100)
    ax.text(-.85,-.7,f"s={h['long']:.3f}m <= .55m\nlateral={h['lateral']:.3f}m <= .28m",fontsize=9)
  elif trajectory:
   path=trajectory.centerline.points;ax.plot([0]+[v.x for v in path],[0]+[v.y for v in path],'g.-',label='after: feasible forward rollout')
   for x,y,yaw in trajectory.poses[::5]:
    R=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]]);box=np.array([[-.09,-.116],[.446,-.116],[.446,.116],[-.09,.116]])@R.T+np.array([x,y]);ax.add_patch(Polygon(box,fill=False,edgecolor='green',alpha=.4))
   ax.text(-.85,-.7,f'free distance={trajectory.free_distance:.2f}m\nbody clearance >= .08m',fontsize=9)
  else:ax.text(-.8,-.7,'No safe forward rollout: STOP retained',fontsize=10)
  fits=r['wall_fits'];bins=sorted(set((x['side'],x['bins']) for x in fits if x['result'] is None));ax.set_title(f"{'Before' if j==0 else 'After'}, sim {r['stamp']:.3f}\nrejected fits: {bins}",fontsize=10)
  ax.set(xlim=(-1,4),ylim=(-1,4),aspect='equal',xlabel='laser X forward (m)',ylabel='laser Y left (m)');ax.grid();ax.legend(fontsize=7,loc='upper right')
 summary.append(dict(stamp=r['stamp'],first_hit=r['corridor_hits'][:1],rejected_fit_bins=bins,safe_forward=None if trajectory is None else dict(steering=trajectory.steering,free_distance=trajectory.free_distance,minimum_clearance=trajectory.minimum_clearance)))
fig.tight_layout();fig.savefig(out/'before_after.png',dpi=160);(out/'geometry_summary.json').write_text(json.dumps(summary,indent=2))
