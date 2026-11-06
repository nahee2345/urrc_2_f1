"""Independent start diagnostics; neither this module nor readiness starts the car."""
import json
from team01_race.start_fallback import StartFallback


def camera_ready(vision_ready, gate_baseline, first_bright_time, rgb_age,
                 depth_good, lidar_good):
    """Only the camera's dark baseline is needed to begin the red sequence.

    Missing Depth/LiDAR does NOT authorize motion: the independent SafetyPlanner
    still refuses commands after LIGHTS_OUT unless both forward sensors are
    fresh and valid. This prevents a deadlock when headless sensor streams
    temporarily do not overlap in time during the light countdown.
    """
    return bool(vision_ready and gate_baseline is not None and
                first_bright_time is None and 0.<=rgb_age<1.5)


class StartDiagnosticsMixin:
    def init_start_diagnostics(self):
        from std_msgs.msg import Bool, String
        self._StartBool=Bool
        self.start_fallback=StartFallback(self.p['start_fallback_delay_s'])
        self.last_sim_time=None
        self._StartString=String
        self.pub_start_ready=self.create_publisher(
            Bool,'/team01/ready_for_lights',10)
        self.pub_start_status=self.create_publisher(
            String,'/team01/debug/start',10)
        self.pub_start_fallback=self.create_publisher(Bool,'/team01/start_fallback',10)

    def authorized(self,now):
        fallback=self.start_fallback.active(now,self.gate)
        self.pub_start_fallback.publish(self._StartBool(data=fallback))
        return bool(fallback or (self.permission and
                    (self.start_mode=='signal' or self.gate.lights_out)))

    def simulated_elapsed_s(self):
        clock=self.get_clock().now().nanoseconds*1e-9
        dt=(max(0.,min(.25,clock-self.last_sim_time))
            if clock>0 and self.last_sim_time is not None else 0.)
        self.last_sim_time=clock if clock>0 else None
        return dt

    def publish_start_diagnostics(self,now,motion,depth_good,lidar_good):
        age=now-self.rgb_at
        status={
            'permission':self.permission,
            'fallback_active':self.start_fallback.active(now,self.gate),
            'spawn_elapsed_s':(None if self.start_fallback.elapsed(now) is None
                               else round(self.start_fallback.elapsed(now),3)),
            'start_source':('CAMERA' if self.permission and self.gate.lights_out
                            else 'NO_RED_7S' if self.start_fallback.active(now,self.gate)
                            else 'NONE'),
            'start_mode':self.start_mode,
            'camera_calibrated':self.signal_vision.ready,
            'camera_frames':self.signal_vision.frames,
            'gate_baseline_ready':self.gate.baseline is not None,
            'gate_status':self.gate.status,
            'red_peak':self.signal_vision.peak_blobs,
            'red_visible':self.rgb_blobs,
            'first_red_seen':self.gate.first_bright_time is not None,
            'lights_out':self.gate.lights_out,
            'camera_age_s':round(age,3),
            'depth_ok':bool(depth_good),
            'front_lidar_ok':bool(lidar_good),
            'motion_mode':motion.mode,
            'motion_reason':motion.reason,
            'cmd_speed_mps':round(float(motion.speed),3),
            'actual_speed_mps':(
                round(float(self.actual_speed),3)
                if self.actual_speed is not None and
                   now-self.odom_at<.6 else None),
        }
        status['ready_for_lights']=camera_ready(
            self.signal_vision.ready,self.gate.baseline,
            self.gate.first_bright_time,age,depth_good,lidar_good)
        self.pub_start_ready.publish(self._StartBool(data=status['ready_for_lights']))
        self.pub_start_status.publish(
            self._StartString(data=json.dumps(status,separators=(',',':'))))
