#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
export PKG_CONFIG_PATH="$(find /opt/ros/jazzy/opt -path '*/lib/pkgconfig' -type d | paste -sd:):${PKG_CONFIG_PATH:-}"
mkdir -p build_f1/telemetry
g++ -O2 -std=c++17 src/team/team_10/tools/contact_recorder.cc -o build_f1/telemetry/contact_recorder $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
g++ -O2 -std=c++17 src/team/team_10/tools/camera_timing.cc -o build_f1/telemetry/camera_timing $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
g++ -O2 -std=c++17 -fPIC -shared src/team/team_10/tools/contact_telemetry.cc -o build_f1/telemetry/libteam10-contact-telemetry.so $(pkg-config --cflags --libs gz-sim8 gz-transport13 gz-msgs10)
