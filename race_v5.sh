#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Both practice map and Gazebo GUI client must share a Gazebo partition.
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
case "${1:-}" in
 follow) shift; exec python3 "$ROOT/race/scripts/follow_v5.py" "$@";;
 map) shift; exec bash "$ROOT/race_v3.sh" "$@";;
 *) echo '사용법: bash race_v5.sh follow --order race/grids/qualifying.txt [--view rear|front]' >&2; exit 2;;
esac
