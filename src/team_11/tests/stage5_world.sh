#!/usr/bin/env bash
set -euo pipefail
TRACK="${1:-monza}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo 'Unknown track' >&2; exit 2;; esac
ROOT="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
PKG="$ROOT/src/urrc_track_gazebo"
if [[ ! -f "$PKG/worlds/${TRACK}_race.sdf" ]]; then PKG="$ROOT"; fi
if [[ ! -f "$PKG/worlds/${TRACK}_race.sdf" ]]; then echo "Missing official track world: $PKG" >&2; exit 2; fi
if [[ ! -f "$PKG/models/urrc_${TRACK}/meshes/road_textured.obj" || ! -f "$PKG/models/urrc_${TRACK}/meshes/asphalt_ground.obj" ]] || ! grep -q 'urrc_v10_spectator' "$PKG/worlds/${TRACK}_race.sdf"; then
  echo "[TEAM11] Official enhanced map assets are missing or incomplete." >&2
  echo "[TEAM11] This launcher will NOT regenerate or modify official map files." >&2
  echo "[TEAM11] To prepare official assets using the upstream tool, close Gazebo and run:" >&2
  echo "  cd $ROOT && bash run_track.sh $TRACK --view grid" >&2
  echo "[TEAM11] That upstream command may generate/update official textures and world files." >&2
  exit 3
fi
export GZ_SIM_RESOURCE_PATH="$PKG/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "Using official unmodified $PKG/worlds/${TRACK}_race.sdf"
exec gz sim -r -v 3 --gui-config "$PKG/worlds/${TRACK}_grid.gui.config" "$PKG/worlds/${TRACK}_race.sdf"
