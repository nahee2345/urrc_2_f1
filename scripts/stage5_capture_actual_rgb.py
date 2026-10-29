#!/usr/bin/env python3
"""0~5단계 Monaco 실제 Gazebo AI-01 RGB 카메라 원본 픽셀 증거 수집.

가짜 프레임, 차선/중심선 정답, 운영자 포즈 입력 금지.
실제 gz topic 이미지 메시지를 수집하고 주행 인식기가 인식하는지 검사.
실제 픽셀 검증 실패 시 CI를 RED로 종료. 단, 이는 3바퀴 시험은 아님.
"""
from __future__ import annotations
import argparse
import ast
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from stage4.lane import estimate_lane, _asphalt


def parse_gz_camera_image(text):
    """gz topic -e 의 실제 gz.msgs.Image 텍스트에서 데이터 필드 해석.

    Gazebo protobuf의 byte string은 \nnn (octal) 또는 escape 문자.
    regex의 마지막 이미지 data는 새로운 메시지 헤더와 구별된다.
    """
    m=re.search(r'\bwidth:\s*(\d+)\s+height:\s*(\d+)',text)
    if not m:
        # Gazebo protobuf 필드는 순서가 환경에 따라 달라질 수 있다.
        mw=re.search(r'\bwidth:\s*(\d+)',text)
        mh=re.search(r'\bheight:\s*(\d+)',text)
        if mw is None or mh is None:
            raise ValueError("Gazebo RGB 메시지에 실제 width/height가 없습니다")
        width,height=int(mw.group(1)),int(mh.group(1))
    else:
        width,height=int(m.group(1)),int(m.group(2))
    if not 40<=width<=4096 or not 30<=height<=4096:
        raise ValueError("비정상 Gazebo 이미지 해상도")
    patterns=(r'\bdata:\s*("(?:\\.|[^"\\])*")',
              r'\bdata:\s*(\'(?:\\.|[^\'\\])*\')')
    raw=None
    for pattern in patterns:
        match=re.search(pattern,text,re.DOTALL)
        if match:
            # protobuf 텍스트 출력은 Python literal과 octal escape 규칙 호환.
            try:
                raw=ast.literal_eval("b"+match.group(1))
            except (SyntaxError,ValueError) as exc:
                raise ValueError("Gazebo 픽셀 bytes 디코딩 실패") from exc
            break
    if raw is None:
        raise ValueError("Gazebo 이미지에 data(bytes)가 없습니다")
    if len(raw)!=width*height*3:
        raise ValueError(
            f"Gazebo RGB 크기 불일치: bytes={len(raw)} expected={width*height*3}."
            " 압축 또는 다른 포맷이라면 검사기 수정이 필요합니다.")
    return width,height,raw


def inspect_rgb(width,height,pixels):
    """원본 실제 프레임에서 검출기 성능과 화면 형태를 수치로 기록."""
    img=estimate_lane(width,height,pixels)
    masks=[]
    ranges=[]
    for f in (.55,.62,.69,.75,.81,.87,.93):
        y=min(height-1,int(height*f))
        count=0
        minx=maxx=None
        for x in range(width):
            k=(y*width+x)*3
            if _asphalt(*pixels[k:k+3]):
                count+=1
                if minx is None:minx=x
                maxx=x
        masks.append(round(count/width,5))
        ranges.append([minx,maxx])
    return {
        "lane_valid":img.valid,"lane_reason":img.reason,
        "lane_error":img.error,"lane_confidence":img.confidence,
        "lane_samples":img.samples,
        "asphalt_fraction_by_image_row":masks,
        "asphalt_bounds_x_by_image_row":ranges,
        "image_width":width,"image_height":height,
        "actual_gazebo_rgb_bytes":len(pixels),
        "source":"gazebo camera /model/urrc_ai_01/sensors/rgb; not simulation ground truth",
    }


def capture(env,topic,seconds):
    with tempfile.TemporaryDirectory(prefix="monaco_rgb_") as d:
        dest=Path(d)/"gazebo_raw_rgb.txt"
        with dest.open("wb") as output:
            proc=subprocess.Popen(["gz","topic","-e","-t",topic],
                                  cwd=ROOT,env=env,stdout=output,
                                  stderr=subprocess.STDOUT,start_new_session=True)
            try:
                time.sleep(seconds)
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    try:proc.wait(timeout=7)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
        return dest.read_text(encoding="utf-8",errors="replace")


def main():
    parser=argparse.ArgumentParser(description="실제 Monaco RGB 원시 Gazebo 카메라 성능 검증")
    parser.add_argument("--output",type=Path,
                        default=ROOT/"generated/stage5/actual_rgb_camera.ppm")
    parser.add_argument("--seconds",type=float,default=5)
    parser.add_argument("--topic",default="/model/urrc_ai_01/sensors/rgb")
    parser.add_argument("--require-lane",action="store_true",
                        help="실제 도로 인식 실패 시 nonzero 종료")
    args=parser.parse_args()
    if not 1<=args.seconds<=15:
        parser.error("1~15초만 수집")
    raw=capture(dict(os.environ),args.topic,args.seconds)
    result={"topic":args.topic,"raw_output_chars":len(raw)}
    try:
        w,h,rgb=parse_gz_camera_image(raw)
        result.update(inspect_rgb(w,h,rgb))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_bytes(f"P6\n{w} {h}\n255\n".encode()+rgb)
        result["image_file"]=str(args.output)
        result["status"]="ACTUAL_RGB_LANE_FOUND" if result["lane_valid"] else "ACTUAL_RGB_LANE_MISSING"
    except ValueError as exc:
        result.update(status="ACTUAL_RGB_UNAVAILABLE",error=str(exc))
    result_file=args.output.with_suffix(".json")
    result_file.parent.mkdir(parents=True,exist_ok=True)
    result_file.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    if result["status"]=="ACTUAL_RGB_UNAVAILABLE":
        return 1
    return 0 if result["lane_valid"] or not args.require_lane else 1


if __name__=="__main__":
    raise SystemExit(main())
