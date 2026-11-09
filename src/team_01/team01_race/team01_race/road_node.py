"""Use geometry-registered wide RGB/Depth and corroborated painted walls."""
import numpy as np
from team01_race.road_surface import (dark_asphalt_bounds,
                                       transverse_start_marking,
                                       aligned_depth_rgb)
from team01_race.rgb_depth_projection import red_white_wall_points


class RoadPerception:
    def refresh(self,node):
        node.road_bounds=None
        node.asphalt_coverage=0.
        node.start_line_seen=False
        node.colored_wall_xy=np.empty((0,2),dtype=float)
        node.wall_color_matches=0
        if (not node.depth_valid or node.road_image is None or
                not node.ground_count or node.depth_shape is None):
            return
        if not aligned_depth_rgb(node.ground_at,node.road_at,
                float(node.actual_speed or 0.),
                float(node.p['road_sync_max_age_s'])):
            return
        depth_stamp=getattr(node,'depth_msg_stamp',0.)
        rgb_stamp=getattr(node,'road_msg_stamp',0.)
        if (depth_stamp>0 and rgb_stamp>0 and
                abs(depth_stamp-rgb_stamp)>
                float(node.p.get('road_frame_sync_tolerance_s',.05))):
            return
        depth_fov=float(node.p['camera_hfov_deg'])
        wide_fov=float(node.p['road_rgb_hfov_deg'])
        transform=dict(depth_shape=node.depth_shape,
                       depth_hfov_deg=depth_fov,rgb_hfov_deg=wide_fov)
        asphalt=dark_asphalt_bounds(
            node.ground_xy,node.ground_uv,node.road_image,**transform)
        node.road_bounds=asphalt.bounds if asphalt.valid else None
        node.asphalt_coverage=asphalt.coverage
        colored=red_white_wall_points(
            node.depth_obstacles,node.obstacle_uv,node.obstacle_height,
            node.road_image,node.depth_shape,depth_fov,wide_fov)
        node.colored_wall_xy=colored
        # A painted RGB object counts as a rail only if raised Depth points
        # spatially agree with actual LiDAR; never turn paint into free road.
        if len(colored) and len(node.lidar_obstacles):
            scan=np.asarray(node.lidar_obstacles,dtype=float).reshape((-1,2))
            agree=np.zeros(len(colored),bool)
            for i in range(0,len(colored),48):
                xy=colored[i:i+48]
                distance2=np.sum((xy[:,None,:]-scan[None,:,:])**2,axis=2)
                agree[i:i+48]=np.min(distance2,axis=1)<(.25)**2
            node.wall_color_matches=int(agree.sum())
        if asphalt.valid:
            node.start_line_seen=transverse_start_marking(
                node.ground_xy,node.ground_uv,node.road_image,
                asphalt.bounds,**transform)
