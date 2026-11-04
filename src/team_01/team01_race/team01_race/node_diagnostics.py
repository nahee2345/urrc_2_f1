"""Bounded diagnostic and first-lap record output."""
import os
import json
import time
from pathlib import Path
import math

class DiagnosticsMixin:
    def _save_rgb(self,rgb,stage):
        try:
            import cv2
            self.camera_debug_dir.mkdir(parents=True,exist_ok=True)
            out=self.camera_debug_dir/f'{stage}.png'
            cv2.imwrite(str(out),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            self._snapshots.add(stage)
        except Exception as exc:
            self.get_logger().warning(f'Camera diagnostic PNG save failed: {exc}')

    def _save_road(self,stage):
        # Frame capture is diagnostic only; it must never authorize movement.
        if self.road_image is None:return
        try:
            import cv2
            self.road_debug_dir.mkdir(parents=True,exist_ok=True)
            from team01_race.road_debug_overlay import overlay_camera
            annotated=overlay_camera(self.road_image,self)
            cv2.imwrite(str(self.road_debug_dir/f'{stage}.png'),annotated)
            self._road_debug_saved.add(stage)
        except Exception as exc:
            self.get_logger().warning(f'Road diagnostic PNG failed: {exc}')

    def _write_learning_progress(self):
        try:
            learning=self.planner.lap_learner
            folder=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state'))) / 'urrc_team01' / 'learning'
            folder.mkdir(parents=True,exist_ok=True)
            dest=folder/'active_lap1.json'
            tmp=folder/'active_lap1.json.tmp'
            tmp.write_text(json.dumps({
                'status':'IN_PROGRESS_NOT_A_COMPLETED_LAP',
                'source':'on-board RGB+Depth, LiDAR, odometry velocity and commanded steering',
                'preloaded_track_map':False,
                'distance_m_est':learning.distance_m,
                'features':learning.first_lap_features,
                'note':'Odometry velocity + steering integration drifts; independent official lap proof is required.',
            },ensure_ascii=False,indent=2),encoding='utf-8')
            tmp.replace(dest)
        except Exception as exc:
            self.get_logger().warning(f'Live track learning snapshot failed: {exc}')

    def _write_lap_record(self):
        if self._lap_record_saved:return
        try:
            learning=self.planner.lap_learner
            folder=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state'))) / 'urrc_team01' / 'learning'
            folder.mkdir(parents=True,exist_ok=True)
            filename=folder/f'lap1_{int(time.time())}.json'
            filename.write_text(json.dumps({
                'status':learning.last_confirmation,
                'note':'Approximate commanded-distance trace, NOT verified Gazebo lap timing',
                'first_lap_distance_m_est':learning.first_lap_distance,
                'first_lap_brake_ticks':learning.first_lap_brakes,
                'records_distance_m_heading_deg_cmd_speed_mode':learning.first_lap_records,
                'observed_track_features':learning.first_lap_features,
                'preloaded_track_map':False,
            },ensure_ascii=False,indent=2),encoding='utf-8')
            self._lap_record_saved=True
            self.get_logger().info(f'Estimated first-lap record saved: {filename}')
        except Exception as exc:
            self.get_logger().warning(f'First-lap record saving failed: {exc}')
