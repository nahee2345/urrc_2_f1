#!/usr/bin/env python3
"""AI-01 Monaco Gazebo 영상/모터/운영자 계측 브리지."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.stage4_bridge import bridge_yaml
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--contacts",type=Path,default=ROOT/"generated/stage5/AI-01/contacts.json")
    p.add_argument("--output",type=Path,default=ROOT/"generated/stage5/AI-01/bridge.yaml")
    a=p.parse_args()
    d=json.loads(a.contacts.read_text())
    if d.get("vehicle")!="urrc_ai_01":raise ValueError("AI-01만 지원")
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(bridge_yaml(d))
    print("Monaco ROS2 브리지 설정:",a.output)
if __name__=="__main__":main()
