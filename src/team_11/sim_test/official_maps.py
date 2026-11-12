"""Download only requested official URRC track assets from the public GitHub repo.

Writes exclusively under urrc_2_f1/build/. Does not touch a local
urrc_f1_ws checkout, original track source, or any team code.
"""
from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path

OFFICIAL_REPO = "https://github.com/nahee2345/urrc_f1_ws.git"
PACKAGE_SUBDIR = Path("src/urrc_track_gazebo")


def _git(args: list[str], *, timeout=900) -> str:
    print("[TEAM11 MAP] git", " ".join(args[:4]), "...", flush=True)
    process=subprocess.run(["git",*args],capture_output=True,text=True,timeout=timeout)
    if process.returncode:
        raise RuntimeError(
            "Official map download failed (git): "+
            (process.stderr or process.stdout)[-800:])
    return process.stdout.strip()


def sync_official_map(workspace: Path, track: str, refresh: bool = False):
    """Return local cache's upstream package path and exact source commit."""
    if track not in ("monza","spa","monaco","cota","hungaroring","baku"):
        raise ValueError("Unrecognized track")
    workspace=Path(workspace).resolve()
    # Never download into or overwrite the separate urrc_f1_ws checkout.
    cache=workspace/"build"/"team11_official_maps_cache"/"urrc_f1_ws"
    if cache.exists():
        if not (cache/".git").exists():
            raise RuntimeError(
                f"Cache exists without Git metadata: {cache}. "
                "Inspect it manually; no files will be replaced.")
        if refresh:
            _git(["-C",str(cache),"fetch","--depth","1","origin","main"])
            _git(["-C",str(cache),"checkout","--force","--detach","FETCH_HEAD"])
    else:
        cache.parent.mkdir(parents=True,exist_ok=True)
        _git(["-c","http.version=HTTP/1.1","clone","--depth","1",
              "--filter=blob:none","--sparse",OFFICIAL_REPO,str(cache)])
    _git(["-C",str(cache),"sparse-checkout","set",
          str(PACKAGE_SUBDIR/"worlds"),
          str(PACKAGE_SUBDIR/"models"/f"urrc_{track}"),
          str(PACKAGE_SUBDIR/"tracks"/"processed")])
    commit=_git(["-C",str(cache),"rev-parse","HEAD"])
    package=cache/PACKAGE_SUBDIR
    needed=[package/"worlds"/f"{track}_race.sdf",
            package/"worlds"/f"{track}_grid.gui.config",
            package/"models"/f"urrc_{track}"/"model.sdf",
            package/"tracks"/"processed"/f"{track}_centerline.csv"]
    missing=[str(p) for p in needed if not p.is_file()]
    if missing:
        raise RuntimeError("Official GitHub map files missing: "+", ".join(missing))
    # The routes in TEAM11 must describe the exact same track mesh geometry.
    bundled=workspace/"src"/"team_11"/"maps"/track/"centerline.csv"
    if not bundled.is_file():
        raise RuntimeError(f"TEAM11 route CSV missing: {bundled}")
    official=needed[-1]
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if sha(official)!=sha(bundled):
        raise RuntimeError(
            "Official centerline has changed since TEAM11 routes were "
            "generated; do NOT drive using stale lane offsets. "
            f"official={official}; bundled={bundled}")
    print("[TEAM11 MAP] Official GitHub revision:",commit,flush=True)
    print("[TEAM11 MAP] Official mesh and bundled centerline: IDENTICAL",
          flush=True)
    print("[TEAM11 MAP] Official mesh directory:",package/"models",flush=True)
    return package,commit
