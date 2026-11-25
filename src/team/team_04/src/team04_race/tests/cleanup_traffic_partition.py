"""Terminate ONLY descendants carrying this exact isolated test partition."""
import argparse,os,signal,json,re
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('partition');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
assert re.fullmatch(r'team04_development_[0-9]+',a.partition)
matched=[]
for d in Path('/proc').iterdir():
 if not d.name.isdigit():continue
 try:
  if d.stat().st_uid!=os.getuid():continue
  env=(d/'environ').read_bytes().split(b'\0')
  if ('GZ_PARTITION='+a.partition).encode() in env:
   pid=int(d.name)
   if pid!=os.getpid():os.kill(pid,signal.SIGINT);matched.append(pid)
 except (OSError,ProcessLookupError,PermissionError):pass
print(json.dumps(dict(partition=a.partition,interrupted_owned_pids=matched)))
a.output.write_text(json.dumps(dict(partition=a.partition,interrupted_owned_pids=matched),indent=2)+'\n')
