#!/usr/bin/env python3
"""Use the unmodified official packer; all generated files stay in team_04."""
from pathlib import Path
import argparse,os,subprocess,sys
p=Path(__file__).resolve().parents[1]
a=argparse.ArgumentParser();a.add_argument('--output',type=Path,default=p.parent/'artifacts/team04.zip');args=a.parse_args()
root=p.parents[2];packer=root/'race/scripts/pack_entry.py'
if not packer.is_file():a.error('Run this helper inside the original URRC repository; official race/scripts/pack_entry.py is required.')
env=os.environ.copy();env['PYTHONDONTWRITEBYTECODE']='1'
subprocess.run([sys.executable,str(packer),str(p),'--output',str(args.output)],check=True,env=env)
