#!/usr/bin/env python3
"""Monaco 5등 RGB 카메라 교정.

1. 실제 신호등 모두 OFF 영상(기준)을 캡처.
2. 빨간불 5개 모두 ON 영상과 비교해 *새로 켜진* 5개만 찾음.
빨간 배경/노란 차체/트랙 장식은 신호등이라고 가정하지 않는다.
소등/점등 5등 증거가 불충분하면 calibration.json을 만들지 않는다.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from stage3.vision import rgb_from_ros
from stage5.roi import detect_five_red_lenses,red_components


def stable_rois(series,threshold=.023):
    if len(series)<3:raise ValueError("교정용 연속 3프레임이 필요합니다")
    checked=series[-3:]
    for cand in checked[1:]:
        if any(abs(v-w)>threshold for a,b in zip(checked[0],cand)
                     for v,w in zip(a,b)):
            raise ValueError("빨간 램프 영역이 프레임마다 움직입니다")
    return [[sum(rows[j][k] for rows in checked)/len(checked)
             for k in range(4)] for j in range(5)]


def save_ppm(path,width,height,rgb):
    path=Path(path)
    if width<1 or height<1 or len(rgb)!=width*height*3:
        raise ValueError("PPM 데이터 크기 오류")
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode()+rgb)


def read_ppm(path):
    try:
        chunks=Path(path).read_bytes().split(b"\n",3)
        if len(chunks)!=4 or chunks[0]!=b"P6" or chunks[2]!=b"255":
            raise ValueError("잘못된 소등 기준 PPM")
        w,h=map(int,chunks[1].split())
        data=chunks[3]
        if w<80 or h<60 or len(data)!=w*h*3:
            raise ValueError("소등 기준 영상 크기가 올바르지 않습니다")
        return w,h,data
    except (OSError,UnicodeError,TypeError) as exc:
        raise ValueError(f"소등 기준 이미지 읽기 실패: {exc}") from exc


def progress_line(received, valid, publishers, error, elapsed, duration,baseline_mode=False):
    if received==0:
        state="ROS 카메라 이미지 미수신"
    elif baseline_mode:
        state="신호등 소등 기준 영상 수집 중"
    elif valid<3:
        state="ROS 이미지 수신, 소등/점등 차분 렌즈 5개 연속 인식 대기"
    else:
        state="ROS 이미지 차분 5등 인식 성공"
    reason=f" / 최근 인식 오류: {error}" if error else ""
    return (f"[출발 카메라] {elapsed:.0f}/{duration:.0f}초 | {state} | "
            f"ROS 발행자 {publishers}개, 수신 {received}프레임, "
            f"연속 유효 {valid}/3{reason}")


def main():
    p=argparse.ArgumentParser(description="실제 ROS 카메라 소등/점등 영상 차분 교정")
    p.add_argument("--topic",default="/model/urrc_ai_01/sensors/start_camera")
    p.add_argument("--output",type=Path,default=ROOT/"generated/stage5/AI-01/calibration.json")
    p.add_argument("--seconds",type=float,default=18)
    p.add_argument("--baseline",type=Path,help="모두 꺼진 카메라 영상(PPM) 파일")
    p.add_argument("--capture-baseline",action="store_true",
                   help="5개 신호등을 켜기 전에 실제 소등 기준 화면만 저장")
    p.add_argument("--diagnostic",type=Path,
                   default=ROOT/"generated/stage5/AI-01/start_camera_diagnostic.ppm")
    a=p.parse_args()
    if not 3<=a.seconds<=90:p.error("3~90초 범위")
    if a.capture_baseline and a.baseline:p.error("기준 영상 캡처와 읽기는 동시에 지정할 수 없음")
    if not a.capture_baseline and not a.baseline:
        p.error("5개 빨간 렌즈 교정에는 실제 --baseline 소등 영상이 필요합니다")
    if a.baseline:
        try:
            bw,bh,off=read_ppm(a.baseline)
        except ValueError as exc:p.error(str(exc))
    else:
        bw=bh=off=None

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
            self.last=None
            self.baseline_frames=[]
            self.create_subscription(Image,a.topic,self.on_image,qos_profile_sensor_data)
        def on_image(self,msg):
            self.attempts+=1
            try:
                w,h,rgb=rgb_from_ros(msg)
                self.last=(w,h,rgb)
                if a.capture_baseline:
                    self.baseline_frames.append((w,h,rgb))
                    self.baseline_frames=self.baseline_frames[-3:]
                    return
                if (w,h)!=(bw,bh):
                    raise ValueError("소등 기준과 점등 카메라 이미지 크기가 다름")
                roi=detect_five_red_lenses(w,h,rgb,baseline=off)
                self.frames.append(roi)
                self.frames=self.frames[-5:]
                if len(self.frames)>=3:
                    stable_rois(self.frames)
                self.error=None
            except ValueError as exc:
                self.error=str(exc)
                self.frames=[]

    rclpy.init()
    node=Capture()
    try:
        start=time.monotonic()
        end=start+a.seconds
        next_report=start
        mode="소등 기준" if a.capture_baseline else "점등 차분"
        print(f"[출발 카메라] {mode} 캡처 | ROS2 토픽: {a.topic}",flush=True)
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            now=time.monotonic()
            if now>=next_report:
                print(progress_line(node.attempts,len(node.frames),
                      node.count_publishers(a.topic),node.error,
                      now-start,a.seconds,baseline_mode=a.capture_baseline),
                      flush=True)
                next_report=now+3
            if a.capture_baseline and len(node.baseline_frames)>=3:
                latest=node.baseline_frames[-1]
                # 카메라가 움직이지 않아야 소등/점등 픽셀 차분이 성립한다.
                w,h,rgb=latest
                old=node.baseline_frames[0][2]
                if len(old)!=len(rgb):
                    raise ValueError("소등 기준 프레임 크기 불일치")
                # 프레임의 평균적인 전체 밝기 변동은 극도로 작아야 함.
                mean_delta=sum(abs(x-y) for x,y in zip(rgb[::12],old[::12]))/len(rgb[::12])
                if mean_delta>12:
                    node.baseline_frames.pop(0)
                    continue
                save_ppm(a.output,w,h,rgb)
                print(f"[출발 카메라] 실제 소등 기준 이미지 저장 완료: {a.output}",flush=True)
                return 0
            if not a.capture_baseline and len(node.frames)>=3:
                rois=stable_rois(node.frames)
                output={"calibrated":True,"source_sensor":"start_camera",
                    "track":"monaco","rois":rois,"stable_frames":3,
                    "calibration_method":"live_rgb_off_on_difference",
                    "reason":"실제 소등/점등 카메라 차분에서 5개 빨간 렌즈 3연속 프레임 확인"}
                a.output.parent.mkdir(parents=True,exist_ok=True)
                a.output.write_text(json.dumps(output,ensure_ascii=False,indent=2)+"\n")
                print("Monaco 실제 출발 신호등 차분 ROI 교정 PASS:",a.output,flush=True)
                return 0

        publishers=node.count_publishers(a.topic)
        if node.last is not None:
            save_ppm(a.diagnostic,*node.last)
            print(f"[카메라 진단] 실제 카메라 사진 저장: {a.diagnostic}",flush=True)
        if node.attempts==0:
            reason=("ROS 이미지 발행자 없음: 브리지·센서 확인" if publishers==0
                    else "ROS 발행자는 있으나 카메라 이미지 수신 실패")
        else:
            reason=node.error or "실제 렌즈 5개를 확인하지 못했습니다"
        print(f"[출발 카메라] 교정 FAIL: 수신 {node.attempts}프레임, "
              f"ROS 발행자 {publishers}개 / {reason}",file=sys.stderr,flush=True)
        return 1
    except KeyboardInterrupt:
        if node.last is not None and not a.capture_baseline:
            save_ppm(a.diagnostic,*node.last)
            print(f"\n[카메라 진단] 중단 직전 출발 카메라 이미지 저장: {a.diagnostic}",flush=True)
        print("[출발 카메라] 사용자 중단. 교정 파일 생성하지 않음",flush=True)
        return 130
    finally:
        node.destroy_node()
        # Jazzy의 SIGINT 핸들러가 이미 rcl_shutdown 호출했다면 두 번째 종료 금지.
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__=="__main__":raise SystemExit(main())
