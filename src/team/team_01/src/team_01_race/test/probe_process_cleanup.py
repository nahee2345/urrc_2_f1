"""Stop only processes inheriting a probe's unique Gazebo partition.

Gazebo can create its own process group; stopping just the CLI group can leave
an orphan physics/render server. Never select user processes by name alone.
"""
import os
from pathlib import Path
import signal
import time


def stop_partition_processes(partition):
    marker=('GZ_PARTITION='+partition).encode()
    def owned():
        result=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
            try:
                if marker in (entry/'environ').read_bytes().split(b'\0'):result.append(int(entry.name))
            except (FileNotFoundError,PermissionError,ProcessLookupError):pass
        return result
    for sig,timeout in ((signal.SIGINT,2.),(signal.SIGTERM,2.),(signal.SIGKILL,.5)):
        targets=owned()
        if not targets:return
        for pid in targets:
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if not owned():return
            time.sleep(.05)
