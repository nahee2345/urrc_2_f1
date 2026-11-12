"""Extract TEAM11 world poses from an isolated vehicle Pose_V -> TF bridge."""
from __future__ import annotations
import math

def yaw_from_quaternion(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def extract_vehicle_pose(msg):
    """Return (x,y,z,yaw,frame), or None if no provable own-model transform.

    A Pose_V can contain more than one transform, not necessarily sorted.
    Prefer the model's world pose over a link pose that might be relative to
    the model. The source topic is TEAM11-private; never use the timer topic.
    """
    candidates=[]
    for tf in msg.transforms:
        name=str(tf.child_frame_id or "").lower().replace("::","/").strip("/")
        parts=name.split("/")
        last=parts[-1] if parts else ""
        parent=str(tf.header.frame_id or "").lower()
        if last=="team_11":
            rank=0
        elif last=="team11":
            rank=1
        elif last=="base_link" and ("team_11" in parts or "team11" in parts):
            rank=2
        elif last=="base_link" and (parent in ("world","map") or parent.startswith("world/")):
            rank=3
        else:
            continue
        pos=tf.transform.translation
        yaw=yaw_from_quaternion(tf.transform.rotation)
        result=(float(pos.x),float(pos.y),float(pos.z),float(yaw),name)
        if all(math.isfinite(v) for v in result[:4]):
            candidates.append((rank,result))
    if not candidates:
        return None
    candidates.sort(key=lambda x:x[0])
    return candidates[0][1]


def pose_frame_examples(msg):
    return [(str(tf.header.frame_id),str(tf.child_frame_id))
            for tf in list(msg.transforms)[:6]]
