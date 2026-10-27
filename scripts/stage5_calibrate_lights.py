#!/usr/bin/env python3
"""Monaco 출발 카메라 실제 다섯 빨간등 모두 ON 상태에서 ROI 자동 교정.

신호등이 보이지 않거나 5개가 정확히 검출되지 않으면 설정 저장 금지.
"""
from __future__ import annotations
import argparse,json,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from stage3.vision import rgb_from_ros
from stage5.roi import detect_five_red_lenses


def stable_rois(series,threshold=.023):
    if len(series)<3:raise ValueError("교정용 연속 3프레임이 필요합니다")
    checked=series[-3:]
    for cand in checked[1:]:
        if any(abs(v-w)>threshold for a,b in zip(checked[0],cand)
                     for v,w in zip(a,b)):
            raise ValueError("빨간 램프 영역이 프레임마다 움직입니다")
    return [[sum(rows[j][k] for rows in checked)/len(checked)
             for k in range(4)] for j in range(5)]


def main():
    p=argparse.ArgumentParser(description="실제 5개 빨간등 자동 ROI 검출")
    p.add_argument("--topic",default="/model/urrc_ai_01/sensors/start_camera")
    p.add_argument("--output",type=Path,default=ROOT/"generated/stage5/AI-01/calibration.json")
    p.add_argument("--seconds",type=float,default=18)
    a=p.parse_args()
    if not 3<=a.seconds<=90:p.error("3~90초 범위")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class Capture(Node):
        def __init__(self):
            super().__init__("urrc_stage5_start_light_calibrator")
            self.frames=[]
            self.attempts=0
            self.error=None
            self.create_subscription(Image,a.topic,self.on_image,qos_profile_sensor_data)
        def on_image(self,msg):
            self.attempts+=1
            try:
                w,h,rgb=rgb_from_ros(msg)
                roi=detect_five_red_lenses(w,h,rgb)
                self.frames.append(roi)
                self.frames=self.frames[-5:]
                if len(self.frames)>=3:
                    stable_rois(self.frames)
            except ValueError as exc:
                self.error=str(exc);self.frames=[]
    rclpy.init()
    node=Capture()
    try:
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(node.frames)>=3:
                rois=stable_rois(node.frames)
                output={"calibrated":True,"source_sensor":"start_camera",
                    "track":"monaco","rois":rois,"stable_frames":3,
                    "reason":"실제 센서 5개 렌즈 3연속 프레임 위치 확인"}
                a.output.parent.mkdir(parents=True,exist_ok=True)
                a.output.write_text(json.dumps(output,ensure_ascii=False,indent=2)+"\n")
                print("Monaco 출발 신호등 ROI 교정 PASS:",a.output)
                return 0
        print(f"Monaco 출발 영상 ROI 교정 FAIL: 수신 {node.attempts}프레임 / {node.error}",
              file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":raise SystemExit(main())
