"""Best-effort read-only spectator telemetry. Never impacts race control."""
from __future__ import annotations
import json
import os
from pathlib import Path
import time
import uuid

def publish(root, track, env, phase, count):
    try:
        path=Path(root)/'race/results/spectator_signal_state.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        state=dict(track=track,partition=env.get('GZ_PARTITION'),
                   phase=phase,count=int(count),timestamp=time.time())
        temporary=path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.tmp')
        temporary.write_text(json.dumps(state)+'\n',encoding='utf-8')
        os.replace(temporary,path)
    except (OSError,TypeError,ValueError) as exc:
        print(f'[관전 신호] 상태 기록 실패 (주행 계속): {exc}',flush=True)
