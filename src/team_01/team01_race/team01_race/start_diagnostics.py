"""Independent start diagnostics; neither this module nor readiness starts the car."""
import json


def camera_ready(vision_ready, gate_baseline, first_bright_time, rgb_age,
                 depth_good, lidar_good):
    """Requires dark image calibration and healthy sensing BEFORE lights start."""
    return bool(vision_ready and gate_baseline is not None and
                first_bright_time is None and 0.<=rgb_age<1.5 and
                depth_good and lidar_good)


class StartDiagnosticsMixin:
    def init_start_diagnostics(self):
        from std_msgs.msg import Bool, String
        self._StartBool=Bool
        self._StartString=String
        self.pub_start_ready=self.create_publisher(
            Bool,'/team01/ready_for_lights',10)
        self.pub_start_status=self.create_publisher(
            String,'/team01/debug/start',10)

    def publish_start_diagnostics(self,now,motion,depth_good,lidar_good):
        age=now-self.rgb_at
        status={
            'permission':self.permission,
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
