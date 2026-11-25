#!/usr/bin/env python3
"""Offline metrics/plots only. Never imported by participant driving code."""
from pathlib import Path
import argparse,json,os
import numpy as np

def analyze(folder,root=None):
 report=json.loads((folder/'result.json').read_text());states=json.loads((folder/'states.json').read_text());poses=np.array(json.loads((folder/'poses.json').read_text()));encoder=np.array(json.loads((folder/'encoder.json').read_text()))
 tracking=[s for s in states if s.get('state')=='TRACKING'];metrics={'trial':folder.name,'completed_laps':report.get('diagnostic_completed_laps',0),'distance_m':report.get('distance_m'),'contacts':len(report.get('contacts',[])),'contact_monitor_messages':report.get('contact_monitor_messages',0),'measured_physical_contacts':report.get('counts',{}).get('physical_contacts',0),'hz_sim':report.get('hz_sim',{}),'tracking_records':len(tracking),'states':{k:sum(s.get('state')==k for s in states) for k in sorted(set(s.get('state','') for s in states))}}
 for key in ['processing_ms','sensor_age_s','target_speed','steering','lateral_error','curvature']:
  v=np.array([s.get(key,0) for s in tracking]);v=v[np.isfinite(v)]
  if len(v):metrics[key]={'mean':float(np.mean(v)),'p95_abs':float(np.percentile(np.abs(v),95)),'max_abs':float(np.max(np.abs(v)))}
 if tracking:metrics['last_valid_fraction']=tracking[-1].get('valid_fraction');metrics['safety_stop_count']=max(s.get('safety_stops',0) for s in tracking)
 if len(encoder):metrics['encoder_moving_speed_mean']=float(np.mean((encoder[:,1]+encoder[:,2])[np.abs(encoder[:,1]+encoder[:,2])>.1]*.035/2))
 reference=None;errors=[]
 if root and len(poses):
  c=np.genfromtxt(root/f"src/urrc_track_gazebo/tracks/processed/{report['track']}_centerline.csv",delimiter=',',names=True);reference=np.stack([c['x_m'],c['y_m']],axis=1)
  for point in poses[:,1:3]:
   nearest=int(np.argmin(np.sum((reference-point)**2,axis=1)));dist=[]
   for i,j in [((nearest-1)%len(reference),nearest),(nearest,(nearest+1)%len(reference))]:
    a,b=reference[i],reference[j];ab=b-a;t=np.clip(np.dot(point-a,ab)/max(1e-12,np.dot(ab,ab)),0,1);dist.append(np.linalg.norm(point-(a+t*ab)))
   errors.append(min(dist))
  metrics['offline_reference_centre_distance']={'mean':float(np.mean(errors)),'p95':float(np.percentile(errors,95)),'max':float(max(errors))}
 (folder/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(2,2,figsize=(13,9))
 if reference is not None:axes[0,0].plot(reference[:,0],reference[:,1],color='.7',lw=2,label='reference (analysis only)')
 if len(poses):axes[0,0].plot(poses[:,1],poses[:,2],lw=1,label='actual vehicle');axes[0,0].set_aspect('equal');axes[0,0].legend()
 axes[0,0].set(title='Observed trajectory',xlabel='world x (m)',ylabel='world y (m)')
 if tracking:
  t=np.array([s['stamp'] for s in tracking]);t0=t[0];t=t-t0
  axes[0,1].plot(t,[s['target_speed'] for s in tracking],label='command speed')
  if len(encoder):axes[0,1].plot(encoder[:,0]-t0,(encoder[:,1]+encoder[:,2])*.035/2,alpha=.5,label='encoder estimate (diagnostic)')
  axes[0,1].set(title='Speed',xlabel='sim time (s)',ylabel='m/s');axes[0,1].legend()
  axes[1,0].plot(t,[s['lateral_error'] for s in tracking],label='local path lateral error');axes[1,0].plot(t,[s['steering'] for s in tracking],label='steering (+right)');axes[1,0].legend();axes[1,0].set(xlabel='sim time (s)',title='Local tracking signals')
  axes[1,1].plot(t,[s['processing_ms'] for s in tracking],label='planning ms');axes[1,1].plot(t,np.array([s['sensor_age_s'] for s in tracking])*1000,label='cloud age ms');axes[1,1].set(xlabel='sim time (s)',ylabel='ms',title='Latency');axes[1,1].legend()
 for ax in axes.flat:ax.grid(alpha=.3)
 fig.tight_layout();fig.savefig(folder/'summary.png',dpi=140);plt.close(fig)
 print(json.dumps(metrics,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--root',type=Path);a=p.parse_args();os.environ.setdefault('MPLCONFIGDIR',str(a.folder/'plot_cache'));analyze(a.folder,a.root)
