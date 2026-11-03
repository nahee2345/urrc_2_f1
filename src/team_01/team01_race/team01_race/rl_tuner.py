"""Safe opt-in table from offline toy-vehicle Q-learning, never online training."""
import json
import math
from pathlib import Path
from functools import lru_cache

ACTIONS=[(l,h,c) for l in (1.25,1.9,2.6)
         for h in (.65,.85,1.05) for c in (.65,1.0,1.35)]


def state(cross,heading,speed):
    return ':'.join(str(v) for v in (
        min(3,int(abs(float(cross))/.16)),
        min(3,int(abs(float(heading))/.10)),
        min(2,int(max(0.,float(speed))/3.))))


@lru_cache(maxsize=4)
def load(path):
    if not path:return {}
    try:
        p=Path(path)
        if not p.is_file() or p.stat().st_size>500000:return {}
        d=json.loads(p.read_text(encoding='utf-8'))
        q=d.get('q',{})
        if d.get('schema')!=1 or not isinstance(q,dict) or len(q)>200:
            return {}
        for k,v in q.items():
            if (not isinstance(k,str) or not isinstance(v,list)
                or len(v)!=len(ACTIONS) or
                not all(isinstance(z,(float,int)) and math.isfinite(z) for z in v)):
                return {}
        return q
    except (OSError,ValueError,TypeError):
        return {}


def action(path,cross,heading,speed):
    row=load(path).get(state(cross,heading,speed))
    if row is None:return None
    return ACTIONS[max(range(len(row)),key=lambda i:row[i])]
