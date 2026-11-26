#!/usr/bin/env bash
# Run a single bounded position or velocity swing-up trial on robot01.
set -euo pipefail

usage() {
    cat <<'HELP'
Usage: bash cartpole_final_push_trial.sh [FRACTION [ENERGY_TRIGGER [SECONDS]]]
       bash cartpole_final_push_trial.sh --velocity [--check-only] [FRACTION [ENERGY_TRIGGER [SECONDS [SPEED_M_S [ACCEL_M_S2]]]]]
Defaults: fraction 0.32, energy 0.83, duration 8 s; velocity cap 0.275 m/s, acceleration 3.5 m/s^2.
--velocity --check-only reads the running node and a fresh prepared reference without moving the cart.
Runs use the adjacent checkout, or CARTPOLE_REPO when set. See docs/VELOCITY_SWING.md.
HELP
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then usage; exit 0; fi
velocity=0
check_only=0
if [[ ${1:-} == --velocity ]]; then
    velocity=1
    shift
    if [[ ${1:-} == --check-only ]]; then check_only=1; shift; fi
fi
if (( $# > 3 + 2 * velocity )); then usage >&2; exit 2; fi
fraction=${1:-0.32}
energy=${2:-0.83}
duration=${3:-8}
[[ $(hostname -s) == robot01 ]] || { echo 'Run on the robot01 host.' >&2; exit 2; }
[[ $check_only == 1 || ${CARTPOLE_BATCH:-0} == 1 || ( -t 0 && -t 1 ) ]] || { echo 'Run in an interactive terminal at the rig or with the operator present.' >&2; exit 2; }

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
default_repo=$HOME/cartpole_ros2
if [[ -d $script_dir/../experiments/cartpole_position_energy_v4 ]]; then default_repo=$script_dir/..; fi
cd "${CARTPOLE_REPO:-$default_repo}"
exec 8>"${XDG_RUNTIME_DIR:-/tmp}/cartpole-swing-$(id -u).lock"
flock -n 8 || { echo 'Another swing trial owns this rig.' >&2; exit 1; }
runner="$PWD/experiments/cartpole_position_energy_v4"
trial=$(mktemp -d /tmp/cartpole-final-push.XXXXXX)
restart_service=0
remote_stage=''
cleanup() {
    local result=$?
    trap - EXIT
    if [[ -n $remote_stage ]]; then
        docker compose exec -T cartpole rm -rf -- "$remote_stage" || true
    fi
    rm -rf -- "$trial"
    if (( restart_service && result == 0 )); then
        docker compose up -d cartpole || result=$?
    fi
    if (( restart_service && result != 0 )); then
        echo 'Check the runner summary and confirm the cart has stopped before restoring ROS.' >&2
        echo 'To restore ROS on the host, run docker compose up -d cartpole.' >&2
    fi
    exit "$result"
}
trap cleanup EXIT

cp "$runner"/*.py "$trial/"
if (( velocity )); then
    velocity_args=(--fraction "$fraction" --energy "$energy" --duration "$duration"
                   --speed "${4:-0.275}" --acceleration "${5:-3.5}")
    # Validate all arguments locally before preparation can move anything.
    python3 "$trial/run_velocity.py" "${velocity_args[@]}"
else
python3 - "$trial/run_position.py" "$fraction" "$energy" "$duration" <<'PY'
import math
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
try:
    fraction, energy, duration = map(float, sys.argv[2:])
except ValueError:
    raise SystemExit("All trial parameters must be numbers.")
for name, value, maximum in (
    ("fraction", fraction, 1.0),
    ("energy trigger", energy, 1.0),
    ("duration", duration, 20.0),
):
    if not math.isfinite(value) or not 0 < value <= maximum:
        raise SystemExit(f"{name} must be > 0 and <= {maximum}.")
source = path.read_text()
if "'final-push'" not in source:
    raise SystemExit("Install the final-push patch first.")
for constant, value in (("FINAL_PUSH_FRACTION", fraction), ("FINAL_PUSH_ENERGY", energy)):
    source, count = re.subn(rf"^{constant}\s*=\s*[^\r\n]+$", f"{constant} = {value!r}", source, flags=re.M)
    if count != 1:
        raise SystemExit(f"Expected one assignment for {constant}.")
compile(source, str(path), "exec")
path.write_text(source)
PY
fi

# Let the container read the staged source even if it runs as a different UID.
chmod 755 "$trial"
chmod a+r "$trial"/*.py
printf 'Final push fraction %s, energy trigger %s, timeout %s s\n' "$fraction" "$energy" "$duration"
compose_input=()
if [[ ${CARTPOLE_BATCH:-0} == 1 || $check_only == 1 ]]; then
    compose_input=(-T)
else
    echo 'Preparation moves the cart. Verify the pole is hanging and the operator has the cutoff.'
    read -r -p 'Type prepare to start preparation: ' approval
    [[ $approval == prepare ]] || { echo 'Cancelled.'; exit 0; }
fi

if (( ! check_only )); then
    docker compose up -d cartpole
    bash tools/cartpole-workflow.sh prepare --pole-down
    docker compose cp "$trial/capture_reference.py" cartpole:/tmp/capture_reference.py
    docker compose exec -T cartpole bash -lc \
        'source /opt/ros/humble/setup.bash && python3 /tmp/capture_reference.py'
fi

if (( velocity )); then
    remote_stage=$(docker compose exec -T cartpole mktemp -d /tmp/cartpole-velocity.XXXXXX)
    [[ $remote_stage =~ ^/tmp/cartpole-velocity\.[[:alnum:]]{6}$ ]] || { remote_stage=''; exit 1; }
    docker compose cp "$trial/." "cartpole:$remote_stage/"
    ros_runner=(docker compose exec -T cartpole bash -lc
        'source /opt/ros/humble/setup.bash && exec python3 "$@"' velocity-trial
        "$remote_stage/run_velocity.py" --pole-down "${velocity_args[@]}")
    "${ros_runner[@]}" --check-only
    if (( check_only )); then exit 0; fi
    if [[ ${CARTPOLE_BATCH:-0} != 1 ]]; then
        read -r -p 'Type run now to execute one velocity trial: ' approval
        [[ $approval == "run now" ]] || { echo 'Trial cancelled.'; exit 0; }
    fi
    "${ros_runner[@]}" --execute
    exit 0
fi
docker compose stop cartpole
restart_service=1

docker compose run --rm --no-deps "${compose_input[@]}" \
    --entrypoint /bin/bash \
    -v "$trial:/profile_test:ro" \
    -v "$PWD/logs:/logs" \
    cartpole -ec '
        cd /profile_test
        python3 run_position.py --check-only --pole-down --policy final-push --duration "$1"
        if [[ $2 != 1 ]]; then
        echo "Confirm the rig is clear. The reference expires 120 s after capture."
        read -r -p "Type run now to execute one trial: " approval
        if [[ $approval != "run now" ]]; then
            echo "Trial cancelled."
            exit 0
        fi
        fi
        exec python3 run_position.py --execute --pole-down --policy final-push --duration "$1"
    ' cartpole-trial "$duration" "${CARTPOLE_BATCH:-0}"
