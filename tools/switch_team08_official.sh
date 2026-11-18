#!/usr/bin/env bash
# In-place switch to official URRC F1 while retaining only TEAM08.
# GitHub origin stays nahee2345/urrc_2_f1 for rollback.
set -Eeuo pipefail
ROOT="${HOME}/urrc_2_f1"
DOWNLOADS="${HOME}/Downloads"
STATE="${DOWNLOADS}/urrc_2_f1_official_rollback.state"
RECOVERY="${DOWNLOADS}/urrc_2_f1_rollback.sh"
ORIGIN="https://github.com/nahee2345/urrc_2_f1.git"
OFFICIAL="https://github.com/nahee2345/urrc_f1_ws.git"
MODE="${1:-}"
fatal() { echo "ERROR: $*" >&2; exit 1; }
[ -d "$ROOT/.git" ] || fatal "기존 ~/urrc_2_f1 Git 저장소가 없습니다."
[ -d "$DOWNLOADS" ] || fatal "~/Downloads 폴더가 없습니다."
command -v git >/dev/null || fatal "git 명령이 없습니다."
command -v tar >/dev/null || fatal "tar 명령이 없습니다."
cd "$ROOT"
if [ "$MODE" = "rollback" ]; then
  [ -f "$STATE" ] || fatal "복구 정보가 없습니다: $STATE"
  IFS= read -r BACKUP < "$STATE"
  [[ "$BACKUP" == "$DOWNLOADS"/urrc_2_f1_before_official_*.tar.gz ]] || fatal "복구 경로가 올바르지 않습니다."
  [ -s "$BACKUP" ] || fatal "백업 파일이 없습니다: $BACKUP"
  echo "[1/4] 기존 GitHub 저장소 재수신"
  git remote set-url origin "$ORIGIN"
  git fetch origin main
  echo "[2/4] GitHub의 기존 urrc_2_f1 상태 복구"
  git reset --hard origin/main
  echo "[3/4] 공식 파일 제거"
  git clean -ffdx
  echo "[4/4] 교체 직전 로컬 파일 전체 복원"
  tar -xzf "$BACKUP" -C "$ROOT"
  echo "복구 완료: $(git rev-parse --short HEAD)"
  git status --short | head -n 30 || true
  echo "백업 파일 보존: $BACKUP"
  exit 0
fi
[ "$MODE" = "apply" ] || fatal "사용법: apply | rollback"
[ ! -e "$STATE" ] || fatal "이전 교체 기록이 있습니다: $STATE"
[ -f src/team/team_08/race_entry.json ] || fatal "TEAM08 제출물이 없습니다."
[ "$(git remote get-url origin)" = "$ORIGIN" ] || fatal "origin이 기대한 GitHub 저장소가 아닙니다."
TEAM_SHA="$(git rev-parse origin/main)"
git cat-file -e "${TEAM_SHA}:src/team/team_08/race_entry.json" || fatal "GitHub 원본에 TEAM08이 없습니다."
echo "[1/6] 공식 urrc_f1_ws 최신 main 확인 (현재 파일 미변경)"
git fetch "$OFFICIAL" main
git cat-file -e FETCH_HEAD:race_v24.sh || fatal "공식 저장소에 race_v24.sh가 없습니다."
git cat-file -e FETCH_HEAD:race/scripts/team_sync_v7.py || fatal "공식 저장소에 팀 동기화 스크립트가 없습니다."
OFFICIAL_SHA="$(git rev-parse FETCH_HEAD)"
BACKUP="${DOWNLOADS}/urrc_2_f1_before_official_$(date +%Y%m%d_%H%M%S).tar.gz"
[ ! -e "$BACKUP" ] || fatal "같은 이름의 백업이 이미 있습니다."
echo "[2/6] 기존 폴더 전체 보존: $BACKUP"
tar -C "$ROOT" -czf "$BACKUP" . || fatal "백업 실패; 작업을 변경하지 않았습니다."
tar -tzf "$BACKUP" >/dev/null || fatal "백업 검증 실패; 작업을 변경하지 않았습니다."
cp -- "${BASH_SOURCE[0]}" "$RECOVERY"
printf '%s\n' "$BACKUP" > "$STATE"
echo "[3/6] 공식 GitHub 내용으로 기존 폴더 제자리 교체"
git reset --hard "$OFFICIAL_SHA"
git clean -ffdx
echo "[4/6] 사용자 GitHub TEAM08 폴더만 복원"
git restore --source="$TEAM_SHA" --worktree -- src/team/team_08
[ -f src/team/team_08/race_entry.json ] || fatal "TEAM08 복원 실패: bash '$RECOVERY' rollback"
echo "[5/6] 교체 결과 확인"
[ -f race_v24.sh ] || fatal "공식 대회 실행 파일 누락"
[ -f race/scripts/vehicle_preflight_v16.py ] || fatal "공식 사전 검사 도구 누락"
python3 -m compileall -q src/team/team_08 || fatal "TEAM08 Python 검사 실패"
python3 - <<'PY'
from pathlib import Path
import json
r=Path('src/team/team_08')
e=json.loads((r/'race_entry.json').read_text())
assert e['entry_id']=='team_08', e
assert (r/e['vehicle_model_file']).is_file(), e['vehicle_model_file']
assert (r/'package.xml').is_file()
assert (r/'launch'/e['algorithm_launch_file']).is_file()
assert len(list((r/'maps').glob('*/track.json')))==6
print('TEAM08 standalone structure PASS')
PY
printf '\n[6/6] 기존 폴더 교체 완료. 공식 커밋 %s, TEAM08 원본 %s\n' "${OFFICIAL_SHA:0:8}" "${TEAM_SHA:0:8}"
echo "실행 전 ROS2 빌드와 공식 사전 검사가 필요합니다."
echo "실패 시: bash '$RECOVERY' rollback"
echo "백업: $BACKUP"
