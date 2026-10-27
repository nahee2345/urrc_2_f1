#!/usr/bin/env python3
"""Monaco 원본 지형·신호등 리소스 유지; AI-01 독립 실험 환경 생성."""
from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from stage5.monaco import generate_monaco
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--ai",default="AI-01")
    p.add_argument("--upstream",type=Path,default=ROOT/"upstream/urrc_f1_ws")
    p.add_argument("--output",type=Path,default=ROOT/"generated/stage5")
    a=p.parse_args()
    result=generate_monaco(a.upstream,a.output,a.ai)
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=="__main__":main()
