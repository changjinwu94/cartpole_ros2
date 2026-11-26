#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
[[ $# == 2 && $1 == prepare && $2 == --pole-down ]] || { echo 'Usage: prepare --pole-down' >&2; exit 2; }
compose=(docker compose -f compose.yaml -f compose.motor.yaml)
"${compose[@]}" cp tools/lqr/. cartpole:/tmp/cartpole_lqr_tools
"${compose[@]}" exec -T cartpole bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec python3 /tmp/cartpole_lqr_tools/prepare_cart.py --pole-down'
