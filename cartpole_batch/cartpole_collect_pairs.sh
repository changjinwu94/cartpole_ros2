#!/usr/bin/env bash
# Collect a bounded A/B series using the existing trial runner.
set -euo pipefail

mode=position
runner_flags=()
if [[ ${1:-} == --velocity ]]; then
    mode=velocity
    runner_flags=(--velocity)
    shift
fi
pairs=${1:-5}
if [[ ! $pairs =~ ^([1-9]|10)$ || $# -gt 3 || ( $mode == position && $# -gt 1 ) ]]; then
    echo 'Usage: bash cartpole_collect_pairs.sh [--velocity] [pairs from 1 to 10] [velocity-only: SPEED_M_S ACCEL_M_S2]' >&2
    exit 2
fi
[[ $(hostname -s) == robot01 ]] || exit 2
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
default_repo=$HOME/cartpole_ros2
if [[ -d $script_dir/../experiments/cartpole_position_energy_v4 ]]; then default_repo=$script_dir/..; fi
repo=${CARTPOLE_REPO:-$default_repo}
export CARTPOLE_REPO="$repo"
runner="$script_dir/cartpole_final_push_trial.sh"
extra=()
if [[ $mode == velocity ]]; then extra=("${2:-0.08}" "${3:-1.0}"); fi
export CARTPOLE_BATCH=1
exec 9>"$repo/.final_push_collection.lock"
flock -n 9 || { echo "Another collection is running." >&2; exit 1; }
[[ -r $runner && -d $repo/logs ]] || { echo 'Trial runner or logs directory missing.' >&2; exit 2; }
prefix=final_push_pairs
if [[ $mode == velocity ]]; then prefix=velocity_final_push_pairs; fi
out=$(mktemp -d "$repo/logs/$prefix.XXXXXX")
header=pair,fraction,outcome,closest_upright_deg,entry_rate_rad_s,stop_confirmed,recording
if [[ $mode == velocity ]]; then header+=,speed_cap_m_s,accel_cap_m_s2,peak_measured_speed_m_s; fi
printf '%s\n' "$header" > "$out/results.csv"
echo "Results: $out"
for ((pair=1; pair<=pairs; pair++)); do
    for fraction in 0.3044 0.3043; do
        transcript="$out/pair${pair}_${fraction}.txt"
        printf '\nPair %s/%s, fraction %s\n' "$pair" "$pairs" "$fraction"
        trial_status=0
        bash "$runner" "${runner_flags[@]}" "$fraction" 0.83 8 "${extra[@]}" </dev/null 2>&1 | tee "$transcript" || trial_status=$?
        python3 - "$transcript" "$repo/logs" "$out/results.csv" "$pair" "$fraction" "$mode" <<'PY'
import csv
import json
import shutil
import sys
from pathlib import Path

transcript, logs, index = map(Path, sys.argv[1:4])
pair, fraction, mode = sys.argv[4:]
for line in reversed(transcript.read_text().splitlines()):
    if line.startswith("RECORDING: /logs/"):
        path = logs / Path(line.split()[-1]).name
        report = json.loads(path.read_text())
        result = report["summary"]
        if result.get("motion_attempted") is True:
            break
else:
    raise SystemExit("Series stopped. No motion trial was recorded.")
saved = f"pair{pair}_{fraction}_{path.name}"
shutil.copy2(path, index.parent / saved)
extra = []
if mode == "velocity":
    if result["control_mode"] != "velocity_final_push_swing_only":
        raise SystemExit("Unexpected control mode in velocity recording")
    cfg = result["settings"]
    extra = [cfg["speed"], cfg["acceleration"],
             max((abs(e["measured_m_s"]) for e in report["events"] if e["kind"] == "velocity_command"), default=None)]
with index.open("a", newline="") as stream:
    csv.writer(stream).writerow([
        pair, fraction, result.get("outcome"), result.get("closest_upright_deg"),
        (result.get("entry_estimate") or {}).get("pole_rate_rad_s"),
        result.get("stop_confirmed"), saved, *extra,
    ])
if result.get("completed") is not True or result.get("stop_confirmed") is not True:
    raise SystemExit("Series stopped. Check the trial and confirm the cart is stopped.")
PY
        if (( trial_status != 0 )); then exit "$trial_status"; fi
    done
done
echo "Completed $pairs pairs. Results: $out/results.csv"
