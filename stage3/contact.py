"""운영자용 실제 접촉 데이터 판정 보조기.

노면 접촉을 사고로 분류하지 않으며, 다른 차량·장벽·장애물과의
실제 Contact 이벤트만 불합격에 반영한다. 토픽 미수신은 무충돌 증거가 아니다.
"""
from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True)
class ContactEvent:
    collision1:str
    collision2:str
    time_s:float

def classify(event:ContactEvent,model:str,road_tokens=("road","ground","asphalt","track")):
    if not (event.collision1.startswith(model+"::") or event.collision2.startswith(model+"::")):
        return "unrelated"
    other=event.collision2 if event.collision1.startswith(model+"::") else event.collision1
    if other.startswith(model+"::"):return "self"
    if any(token in other.lower() for token in road_tokens):
        return "road"
    if any(token in other.lower() for token in ("wall","barrier","obstacle","cone")):
        return "wall"
    return "vehicle" if "car" in other.lower() or "vehicle" in other.lower() else "unknown_collision"
