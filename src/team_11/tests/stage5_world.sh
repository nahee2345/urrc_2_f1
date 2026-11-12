#!/usr/bin/env bash
set -euo pipefail
TRACK="${1:-monza}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo 'Unknown track' >&2; exit 2;; esac
ROOT="${URRC_OFFICIAL_WS:-$HOME/urrc_f1_ws}"
PKG="$ROOT/src/urrc_track_gazebo"
if [[ ! -f "$PKG/worlds/${TRACK}_race.sdf" ]]; then PKG="$ROOT"; fi
if [[ ! -f "$PKG/worlds/${TRACK}_race.sdf" ]]; then echo "Missing official track world: $PKG" >&2; exit 2; fi
export GZ_SIM_RESOURCE_PATH="$PKG/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "Using official unmodified $PKG/worlds/${TRACK}_race.sdf"
exec gz sim -r -v 3 --gui-config "$PKG/worlds/${TRACK}_grid.gui.config" "$PKG/worlds/${TRACK}_race.sdf"
