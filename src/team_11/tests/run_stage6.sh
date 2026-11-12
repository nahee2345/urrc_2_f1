#!/usr/bin/env bash
set -euo pipefail
# Stage 6 one-command local official-world check (passes context to stage5 integrated runner).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HERE/run_stage5.sh" "${1:-monza}" "${2:-1}"
