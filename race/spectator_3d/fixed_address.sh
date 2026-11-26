#!/usr/bin/env bash
set -euo pipefail
command -v avahi-publish >/dev/null || { echo 'sudo apt install avahi-daemon avahi-utils'; exit 2; }
IP="${1:-$(ip route get 192.0.2.1 | awk '{for(i=1;i<=NF;i++)if($i=="src"){print $(i+1);exit}}')}"
[[ "$IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ && "$IP" != 127.* ]] || { echo '사용법: bash spectator/fixed_address.sh 운영PC_LAN_IP'; exit 2; }
echo "urrc2.local -> $IP (이 터미널이 실행 중일 때 유지)"
exec avahi-publish -a -R urrc2.local "$IP"
