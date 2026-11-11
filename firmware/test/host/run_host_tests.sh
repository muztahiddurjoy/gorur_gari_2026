#!/usr/bin/env bash
# Host-side tests for the firmware's pure-logic modules: compiled with the
# system g++ against stub Arduino headers (test/host/stubs), no board needed.
#
#   firmware/test/host/run_host_tests.sh
#
# PlatformIO's own test runner ignores this directory (it only picks up
# test/test_* folders), so `pio run` / `pio test` are unaffected.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(cd "$HERE/../.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -Wall -Wextra -Werror -O1
       -I "$HERE" -I "$HERE/stubs" -I "$FW/include" -I "$FW/lib/mav_msgs")

status=0
build_and_run() {
    local name="$1"; shift
    echo "== $name"
    if "$CXX" "${FLAGS[@]}" "$@" -o "$OUT/$name"; then
        "$OUT/$name" || status=1
    else
        echo "   build failed"; status=1
    fi
}

build_and_run test_motor "$HERE/test_motor.cpp" "$FW/lib/motor_control/motor_control.cpp" \
    -I "$FW/lib/motor_control"
build_and_run test_steering "$HERE/test_steering.cpp" "$FW/lib/steering_control/steering_control.cpp" \
    -I "$FW/lib/steering_control"
build_and_run test_drive_link "$HERE/test_drive_link.cpp" -I "$FW/lib/drive_link"
build_and_run test_yaw "$HERE/test_yaw.cpp" -I "$FW/lib/heading"

exit $status
