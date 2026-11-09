"""Measured vehicle-frame road corridor diagnostics; never rasterize as BEV."""
import json
import numpy as np
from team01_race.road_roi import road_roi_metrics


def measured_corridor(road, obstacles, *, half_width=.18):
    result={'frame_id':'base_link','representation':'metric_corridor',
            'valid':False,'stations':[],'nearest_obstacle_m':None,
            'roi':road_roi_metrics(road)}
    if road is None or not road.valid or road.bounds is None:return result
    b=np.asarray(road.bounds,dtype=float)
    if (b.ndim!=2 or b.shape[1]!=3 or len(b)<4 or
            not np.all(np.isfinite(b)) or not np.all(np.diff(b[:,0])>0) or
            not np.all(b[:,2]>b[:,1])):return result
    for x in (1.,2.,3.,5.,8.,10.):
        if not (b[0,0]<=x<=b[-1,0]):continue
        right=float(np.interp(x,b[:,0],b[:,1]))
        left=float(np.interp(x,b[:,0],b[:,2]))
        result['stations'].append({
            'forward_m':x,'right_m':round(right,3),
            'left_m':round(left,3),
            'usable_width_m':round(max(0.,left-right-2.*half_width-.08),3)})
    result['valid']=bool(result['stations'])
    pts=np.asarray(obstacles,dtype=float).reshape((-1,2))
    if len(pts):
        pts=pts[np.all(np.isfinite(pts),axis=1)]
        pts=pts[(pts[:,0]>.28)&(pts[:,0]>=b[0,0])&(pts[:,0]<=b[-1,0])]
        if len(pts):
            right=np.interp(pts[:,0],b[:,0],b[:,1])
            left=np.interp(pts[:,0],b[:,0],b[:,2])
            inside=(pts[:,1]>=right)&(pts[:,1]<=left)
            if np.any(inside):
                result['nearest_obstacle_m']=round(float(np.min(
                    np.hypot(pts[inside,0],pts[inside,1]))),3)
    return result


class CorridorPublisher:
    def init_corridor_debug(self):
        from std_msgs.msg import String
        self._CorridorString=String
        self.pub_corridor=self.create_publisher(
            String,'/team01/debug/corridor',2)
        self.pub_shadow_mpc=self.create_publisher(
            String,'/team01/debug/mpc_shadow',2)
        self.last_corridor_at=0.

    def publish_corridor_debug(self,now):
        if now-self.last_corridor_at<.10:return
        obstacles=np.vstack((self.depth_obstacles,self.lidar_obstacles))
        data=measured_corridor(self.planner.road,obstacles,
            half_width=float(self.p['path_half_width_m']))
        data['road_source']=self.planner.road_source
        data['motion_mode']=self.planner.last_mode
        fresh=bool(now-self.depth_at<float(self.p['depth_timeout_s']) and
                   now-self.lidar_at<float(self.p['lidar_timeout_s']))
        data['source_sensor_fresh']=fresh
        if not fresh:
            data.update(valid=False,stations=[],nearest_obstacle_m=None)
        self.pub_corridor.publish(self._CorridorString(
            data=json.dumps(data,separators=(',',':'))))
        if self.p.get('enable_shadow_mpc',False):
            from team01_race.sampled_mpc import search_shadow_mpc
            proposal=search_shadow_mpc(
                self.planner.road,self.planner.reference_geometry,obstacles,
                self.actual_speed if self.actual_speed is not None
                else self.planner.speed,self.planner.prev_steer,self.p)
            payload={'feasible':proposal is not None,
                     'steer_rad':None if proposal is None else
                                 round(proposal.steer_rad,5),
                     'clearance_m':None if proposal is None else
                                   round(proposal.min_clearance_m,3),
                     'cost':None if proposal is None else
                            round(proposal.cost,4),'actuator_enabled':False}
            self.pub_shadow_mpc.publish(self._CorridorString(
                data=json.dumps(payload,separators=(',',':'))))
        self.last_corridor_at=now
