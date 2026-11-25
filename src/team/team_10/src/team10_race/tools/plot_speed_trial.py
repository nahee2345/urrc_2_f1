#!/usr/bin/env python3
"""Offline speed evidence plot; no live interfaces."""
import argparse,csv,gzip,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--run',required=True);a=p.parse_args();out=Path(a.run)
with gzip.open(out/'speed_timeline.csv.gz','rt') as f:rows=list(csv.DictReader(f))
m=json.loads((out/'speed_metrics.json').read_text());t0=m['camera_release_sim'];end=m['end_sim']
rows=[r for r in rows if t0<=float(r['sim'])<=end]
t=np.array([float(r['sim'])-t0 for r in rows])
fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True,constrained_layout=True)
for key,label in [('actual_mps','Actual (0.2s pose mean)'),('adapter_mps','Adapter'),('target','Planner target')]:
    axes[0].plot(t,[float(r[key]) for r in rows],label=label,linewidth=1)
axes[0].axhline(10,color='red',ls='--',lw=1,label='10 m/s')
axes[0].set_ylabel('Speed (m/s)');axes[0].legend(ncol=4,fontsize=8);axes[0].grid(alpha=.25)
for key,label in [('cap_CURVATURE','Curvature'),('cap_STEERING','Steering'),('cap_VISIBILITY','Verified visibility')]:
    axes[1].plot(t,[float(r[key]) for r in rows],label=label,lw=.8)
axes[1].set_ylabel('Candidate cap (m/s)');axes[1].legend(ncol=3,fontsize=8);axes[1].grid(alpha=.25)
axes[2].plot(t,[float(r['filtered_left']) for r in rows],label='Commanded steering (+left)',lw=.8)
axes[2].plot(t,[float(r['actual_steering_left']) if r['actual_steering_left'] else np.nan for r in rows],label='Measured equivalent steering',lw=.8)
axes[2].set_ylabel('Steering (rad)');axes[2].set_xlabel('Sim seconds since optical release (initial acceleration included)')
axes[2].legend(fontsize=8);axes[2].grid(alpha=.25)
fig.suptitle(out.name+' | mean %.2f m/s | >=10: %.3f sim s (%.2f%%)'%(m['mean_actual_mps'],m['above_10_sim_s'],100*m['above_10_ratio']))
fig.savefig(out/'speed_evidence.png',dpi=160);plt.close(fig)
