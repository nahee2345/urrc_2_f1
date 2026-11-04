"""ROS-facing publisher of conservative 10Hz local BEV for RViz inspection."""
import numpy as np
from team01_race.bev_grid import local_bev


class BEVPublisher:
    def init_bev(self):
        from nav_msgs.msg import OccupancyGrid
        self._OccupancyGrid=OccupancyGrid
        self.pub_bev=self.create_publisher(
            OccupancyGrid,'/team01/debug/bev',2)
        self.last_bev_at=0.
        from std_msgs.msg import String
        self._MPCString=String
        self.pub_shadow_mpc=self.create_publisher(
            String,'/team01/debug/mpc_shadow',2)

    def publish_bev(self,now):
        if now-self.last_bev_at<.10:return
        grid=local_bev(self.planner.road,
                       np.vstack((self.depth_obstacles,self.lidar_obstacles)))
        msg=self._OccupancyGrid()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='base_link'
        msg.info.resolution=.2
        msg.info.width=int(grid.shape[1])
        msg.info.height=int(grid.shape[0])
        msg.info.origin.position.y=-3.0
        msg.info.origin.orientation.w=1.0
        msg.data=grid.flatten().astype(int).tolist()
        self.pub_bev.publish(msg)
        if self.p.get('enable_shadow_mpc',False):
            from team01_race.sampled_mpc import search_shadow_mpc
            import json
            proposal=search_shadow_mpc(
                self.planner.road,self.planner.reference_geometry,
                np.vstack((self.depth_obstacles,self.lidar_obstacles)),
                self.actual_speed if self.actual_speed is not None
                else self.planner.speed,
                self.planner.prev_steer,self.p)
            payload={'feasible':proposal is not None,
                     'steer_rad':None if proposal is None else
                                  round(proposal.steer_rad,5),
                     'clearance_m':None if proposal is None else
                                   round(proposal.min_clearance_m,3),
                     'cost':None if proposal is None else
                            round(proposal.cost,4),
                     'actuator_enabled':False}
            self.pub_shadow_mpc.publish(self._MPCString(
                data=json.dumps(payload,separators=(',',':'))))
        self.last_bev_at=now
