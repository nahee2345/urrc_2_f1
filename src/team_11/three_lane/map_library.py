"""Portable TEAM 11 practice-route library (not an official race entry).

Bundled centerline and grid files are sourced from the unchanged organizer
track data. The package needs no external centerline repository at runtime.
Track identity must be positively established; never guess a map.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re

from core import TRACKS, export_routes, lane_spacing, load_centerline

HERE = Path(__file__).resolve().parent
MAPS = HERE / "maps"


class UnknownTrack(ValueError):
    pass


def normalize_track(value: str | None) -> str | None:
    if value is None:
        return None
    name = Path(str(value).strip().lower()).stem
    name = re.sub(r"(_race|_grid|_team11|_world)$", "", name)
    return name if name in TRACKS else None


def map_directory(track: str) -> Path:
    canonical = normalize_track(track)
    if canonical is None:
        raise UnknownTrack(f"Unsupported track: {track!r}")
    directory = MAPS / canonical
    if not (directory / "centerline.csv").is_file():
        raise FileNotFoundError(f"Missing bundled centerline: {directory}")
    if not (directory / "grid.json").is_file():
        raise FileNotFoundError(f"Missing bundled grid metadata: {directory}")
    return directory


def default_state_file() -> Path:
    state_home = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state")))
    return state_home / "urrc_f1" / "last_run.json"


def track_from_spawn(x: float, y: float, z: float, yaw: float) -> str | None:
    """Match a known spawn pose conservatively; ambiguity means no match."""
    choices = []
    for track in TRACKS:
        data = json.loads((map_directory(track) / "grid.json").read_text())
        for grid in data["grid"][:12]:
            distance = math.hypot(x - grid["x"], y - grid["y"])
            elevation = abs(z - grid["road_z"])
            turn = abs(math.atan2(math.sin(yaw - grid["yaw"]),
                                  math.cos(yaw - grid["yaw"])))
            if distance < 0.09 and elevation < 0.12 and turn < 0.12:
                choices.append((distance + elevation + turn, track))
    choices.sort()
    if not choices or (len(choices) > 1 and choices[1][1] != choices[0][1]):
        return None
    return choices[0][1]


def detect_track(*, explicit: str | None = None, world_name: str | None = None,
                 world_file: str | None = None, state_file: str | Path | None = None,
                 spawn: tuple[float, float, float, float] | None = None) -> str:
    """Resolve a map from supplied world metadata or an operator run manifest.

    The state file is optional; stale/unknown data is not treated as a match.
    """
    evidence: list[tuple[str, str]] = []
    for source, value in (("explicit", explicit), ("world_name", world_name),
                          ("world_file", world_file)):
        if value:
            match = normalize_track(value)
            if match is None:
                raise UnknownTrack(f"Unknown map from {source}: {value!r}")
            evidence.append((source, match))
    path = Path(state_file) if state_file is not None else default_state_file()
    # Only use persistent state when a track was not explicitly provided.
    if not evidence and path.is_file():
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("track"), str):
            raise UnknownTrack("Invalid last_run.json map identifier")
        match = normalize_track(data["track"])
        if match is None:
            raise UnknownTrack(f"Unsupported last-run map: {data['track']!r}")
        evidence.append(("state_file", match))
    if not evidence and spawn is not None:
        matched = track_from_spawn(*spawn)
        if matched:
            evidence.append(("spawn_pose", matched))
    if not evidence:
        raise UnknownTrack(
            "Map not identified; pass --track or --world-name, "
            "or provide a trusted last_run.json. No default-track fallback."
        )
    if len({name for _, name in evidence}) != 1:
        raise UnknownTrack(f"Conflicting map identity: {evidence!r}")
    return evidence[0][1]


def build_route_bundle(track: str, output: str | Path) -> dict:
    canonical = detect_track(explicit=track)
    base = map_directory(canonical)
    points = load_centerline(base / "centerline.csv")
    if len(points) < 50:
        raise ValueError("Route has too few samples")
    spacing = lane_spacing(points)
    if spacing < 0.26:
        raise ValueError("Lane spacing unsafe for 0.25m vehicle")
    destination = Path(output).expanduser().resolve() / canonical
    destination.mkdir(parents=True, exist_ok=True)
    export_routes(points, destination)
    grid = json.loads((base / "grid.json").read_text(encoding="utf-8"))
    lap_length = points[-1].s + math.hypot(points[-1].x - points[0].x,
                                          points[-1].y - points[0].y)
    manifest = {
        "track": canonical,
        "lap_length_m": lap_length,
        "lane_spacing_m": spacing,
        "sample_count": len(points),
        "grid_count": len(grid["grid"]),
        "route_prefix": str(destination),
        "source_commit": grid["source_commit"],
        "usage": "practice_only; pre-recorded track driving is prohibited by organizer rules",
    }
    (destination / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline TEAM 11 map-route packaging")
    parser.add_argument("--track", default="auto", help="auto or one of: " + ", ".join(TRACKS))
    parser.add_argument("--world-name", help="Trusted current world name")
    parser.add_argument("--world-file", help="Trusted current world filename")
    parser.add_argument("--state-file", help="Operator last_run.json")
    parser.add_argument("--output", default=str(HERE / "generated_routes"))
    args = parser.parse_args()
    try:
        selected = detect_track(
            explicit=None if args.track == "auto" else args.track,
            world_name=args.world_name,
            world_file=args.world_file,
            state_file=args.state_file)
        print(json.dumps(build_route_bundle(selected, args.output),
                         ensure_ascii=False, indent=2))
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
