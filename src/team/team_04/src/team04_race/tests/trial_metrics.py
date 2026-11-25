"""Stage3 observer-only measurement. Never used by the driving nodes."""
import csv,json,os,time
from pathlib import Path

class TrialMetrics:
 def __init__(self):
  self.release_sim=None;self.release_wall=None;self.release_progress=0.;self.release_pose=None
  self.laps=[];self.last_pose=None;self.paths=[];self.commands=[];self.stats=[];self.performance=[];self.latest_command=[0.,0.];self.last_cpu=None;self.qualifying_distance=False;self.approach_distance=0.;self.approach_crossing=None;self.armed_wall=None
 def released(self,stamp,progress,pose):
  self.release_sim=stamp;self.release_progress=progress+(self.approach_distance if self.qualifying_distance else 0.);self.release_pose=pose;self.armed_wall=time.monotonic()
 def resumed(self):self.release_wall=time.monotonic()
 def pose(self,stamp,progress,pose,length):
  if self.release_sim is not None and self.last_pose:
   previous_stamp,previous_progress=self.last_pose
   relative=progress-self.release_progress
   if self.qualifying_distance and self.approach_crossing is None and relative>=0:
    fraction=max(0.,min(1.,(self.release_progress-previous_progress)/max(1e-9,progress-previous_progress)));self.approach_crossing=previous_stamp+fraction*(stamp-previous_stamp)
   while relative>=(len(self.laps)+1)*length:
    target=self.release_progress+(len(self.laps)+1)*length
    fraction=max(0.,min(1.,(target-previous_progress)/max(1e-9,progress-previous_progress)))
    crossing=previous_stamp+fraction*(stamp-previous_stamp)
    previous=self.laps[-1]['crossing_sim'] if self.laps else (self.approach_crossing if self.qualifying_distance and self.approach_crossing is not None else self.release_sim)
    self.laps.append(dict(lap=len(self.laps)+1,crossing_sim=crossing,lap_seconds=crossing-previous,elapsed_seconds=crossing-self.release_sim,position=pose[1:4],wall_elapsed_from_resume_s=time.monotonic()-self.release_wall if self.release_wall else None,wall_elapsed_from_arm_s=time.monotonic()-self.armed_wall if self.armed_wall else None))
  self.last_pose=(stamp,progress)
 def path(self,msg):
  stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
  if not self.paths or stamp-self.paths[-1]['stamp']>=.099:
   self.paths.append(dict(stamp=stamp,frame=msg.header.frame_id,points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]))
 def command(self,stamp,value,steering=False):
  self.latest_command[1 if steering else 0]=float(value)
  if not steering:self.commands.append([stamp,*self.latest_command])
 def world_stats(self,msg):
  stamp=msg.sim_time.sec+msg.sim_time.nanosec*1e-9
  self.stats.append([stamp,float(msg.real_time_factor),bool(msg.paused)])
 @staticmethod
 def process_usage(process):
  pending=[process.pid];seen=set();cpu=0.;rss=0
  while pending:
   pid=pending.pop()
   if pid in seen:continue
   seen.add(pid)
   try:
    path=Path('/proc')/str(pid);fields=(path/'stat').read_text().split(') ',1)[1].split()
    cpu+=(float(fields[11])+float(fields[12]))/os.sysconf('SC_CLK_TCK')
    rss+=int((path/'statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
    pending.extend(map(int,(path/'task'/str(pid)/'children').read_text().split()))
   except (OSError,ValueError,IndexError):pass
  return cpu,rss/1048576
 def checkpoint(self,sim,progress,processes):
  wall=time.monotonic();cpu=0.;rss=0.
  for process in processes:
   c,r=self.process_usage(process);cpu+=c;rss+=r
  percent=0. if self.last_cpu is None else max(0.,100*(cpu-self.last_cpu[1])/max(.001,wall-self.last_cpu[0]))
  self.last_cpu=(wall,cpu)
  row=dict(sim=sim,wall_elapsed=wall-self.release_wall if self.release_wall else 0.,progress_m=progress,cpu_percent_one_core=percent,process_rss_sum_mb=rss,rtf=self.stats[-1][1] if self.stats else None)
  self.performance.append(row);return row
 @staticmethod
 def csv(path,rows,fields=None):
  if not rows:return
  if isinstance(rows[0],dict):
   fields=fields or list(rows[0]);f=path.open('w',newline='');w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows);f.close()
  else:
   with path.open('w',newline='') as f:w=csv.writer(f);w.writerow(fields or []);w.writerows(rows)
 def save(self,out,report):
  elapsed=(report.get('end_sim',0)-self.release_sim) if self.release_sim is not None else None
  report['legacy_progress_laps']=report.get('diagnostic_completed_laps',0)
  if self.release_sim is not None:report['diagnostic_completed_laps']=len(self.laps)
  report.update(qualifying_distance=self.qualifying_distance,approach_distance_m=self.approach_distance if self.qualifying_distance else None,approach_sim_time=self.approach_crossing,release_sim_time=self.release_sim,driving_sim_seconds=elapsed,lap_records=self.laps,lap_times_seconds=[v['lap_seconds'] for v in self.laps],rtf_mean=sum(v[1] for v in self.stats)/len(self.stats) if self.stats else None)
  if self.release_pose and report.get('last_pose'):
   import math
   report['return_xy_distance_m']=math.dist(self.release_pose[1:3],report['last_pose'][1:3])
  (out/'paths.json').write_text(json.dumps(self.paths));(out/'commands.json').write_text(json.dumps(self.commands));(out/'performance.json').write_text(json.dumps(self.performance));(out/'world_stats.json').write_text(json.dumps(self.stats))
  self.csv(out/'laps.csv',self.laps,['lap','crossing_sim','lap_seconds','elapsed_seconds','position','wall_elapsed_from_resume_s','wall_elapsed_from_arm_s'])
  self.csv(out/'commands.csv',self.commands,['stamp','speed','steering'])
  self.csv(out/'performance.csv',self.performance)
  self.csv(out/'world_stats.csv',self.stats,['stamp','rtf','paused'])

class ExistingStartObserver:
 """Observe the unmodified official light/start sequence; no timing publications."""
 def __init__(self,node,last,metrics,get_progress,get_pose):self.node=node;self.last=last;self.metrics=metrics;self.get_progress=get_progress;self.get_pose=get_pose
 def frozen_time(self):
  import rclpy
  end=time.monotonic()+.15
  while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.01)
  if self.last.get('clock',0)<=0:raise RuntimeError('central test clock unavailable')
  return self.last['clock']
 def arm(self,ids,laps,stamp):self.metrics.released(stamp,self.get_progress(),self.get_pose())
 def note_resume(self):self.metrics.resumed()


def classify_safety_events(states,release):
 active=[];before=[];previous=None;started=False
 for state in states:
  if state.get('state')=='TRACKING' and state.get('stamp',0)>release+1e-4 and state.get('target_speed',0)>0:started=True
  reason=state.get('state')
  if reason not in ('TRACKING','WAITING_FOR_START','STARTUP') and reason!=previous:
   (active if started else before).append(state)
  previous=reason
 return active,before
