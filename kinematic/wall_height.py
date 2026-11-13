"""Lower the SIX official-copy wall meshes to urrc_2's 0.30m height.

Only generated run resources are modified. Road and car geometry are untouched.
Both collision and coloured wall visual OBJ meshes receive the same Z change.
"""
from pathlib import Path
import os
import shutil

OFFICIAL_HEIGHT_M=.80
PRACTICE_HEIGHT_M=.30
DELTA=OFFICIAL_HEIGHT_M-PRACTICE_HEIGHT_M
MESH_NAMES=(
    "wall_inner.obj","wall_outer.obj",
    "wall_inner_collision.obj","wall_outer_collision.obj",
    "wall_inner_red.obj","wall_inner_white.obj",
    "wall_outer_red.obj","wall_outer_white.obj",
)

def _copy_or_link(source,target):
    try:
        os.link(source,target)
    except OSError:
        shutil.copy2(source,target)

def _vertices(path):
    """Build each wall's low-Z reference for identical XY vertex positions."""
    lows={}
    highs={}
    with path.open("r",encoding="ascii") as stream:
        for line in stream:
            if not line.startswith("v "):
                continue
            _,sx,sy,sz=line.split()
            key=(sx,sy)
            z=float(sz)
            lows[key]=min(lows.get(key,z),z)
            highs[key]=max(highs.get(key,z),z)
    if not lows:raise ValueError("Wall mesh has no vertices: "+str(path))
    # The official extrusion uses 0.8m above and 0.01m below road surface.
    # Refuse a structurally different wall rather than shifting road material.
    counts=sum(1 for xy in lows if abs(highs[xy]-lows[xy]-
                                     (OFFICIAL_HEIGHT_M+.01))<.03)
    if counts<.75*len(lows):
        raise ValueError(f"Unexpected original wall extrusion: {path}; paired={counts}/{len(lows)}")
    return lows,highs

def _lower_mesh(src,dst,lows,highs):
    dst.parent.mkdir(parents=True,exist_ok=True)
    modified=0
    with src.open("r",encoding="ascii") as reader, dst.open("w",encoding="ascii") as writer:
        for line in reader:
            if line.startswith("v "):
                _,sx,sy,sz=line.split()
                key=(sx,sy)
                z=float(sz)
                if key not in lows:
                    raise ValueError(f"Missing wall reference XY in {src}: {key}")
                if highs[key]-lows[key]>.5 and z-lows[key]>.5:
                    z-=DELTA
                    modified+=1
                line=f"v {sx} {sy} {z:.8f}\n"
            writer.write(line)
    if modified==0:
        raise ValueError("No wall upper vertices changed: "+str(src))
    return modified

def prepare_wall_resources(package,runtime,track):
    """Generate independent local model://urrc_<track> wall resources."""
    source=Path(package)/"models"/f"urrc_{track}"
    target=Path(runtime)/"models"/f"urrc_{track}"
    if not source.is_dir():
        raise FileNotFoundError(source)
    (target/"meshes").mkdir(parents=True,exist_ok=True)
    for p in source.rglob("*"):
        if not p.is_file():continue
        dst=target/p.relative_to(source)
        dst.parent.mkdir(parents=True,exist_ok=True)
        if p.name not in MESH_NAMES:
            _copy_or_link(p,dst)
    result={}
    for wall in ("wall_inner","wall_outer"):
        reference=source/"meshes"/f"{wall}.obj"
        lows,highs=_vertices(reference)
        for name in (f"{wall}.obj",f"{wall}_collision.obj",
                     f"{wall}_red.obj",f"{wall}_white.obj"):
            result[name]=_lower_mesh(source/"meshes"/name,
                                     target/"meshes"/name,lows,highs)
    # Keep original model.sdf mesh refs; generated resource directory
    # is prepended to Gazebo's model lookup path by run.sh.
    if sorted(result)!=sorted(MESH_NAMES):
        raise ValueError("Did not transform all eight wall OBJ meshes")
    print(f"[WALL] {track}: original 0.80m -> practice 0.30m; "
          f"8 visual/collision meshes updated; road unchanged",flush=True)
    return result
