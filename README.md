# ROS 2 cartpole swing-up on Raspberry Pi

This repository runs position-target swing-up experiments on the physical cartpole at `robot01`. An Arduino UNO R4 Minima reads the pole encoder and endstops. A Raspberry Pi 4 running Ubuntu 22.04 hosts ROS 2 Humble in Docker and communicates with the Faulhaber motor controller over RS-232. The experiment measures how the final cart move changes the pole's approach to upright. The current runner stops on entry into a 12° upright window. Active balance and a five-second hold remain development goals.

## Set up a checkout

On a fresh host with Git, Python 3, Docker Engine and Docker Compose installed, clone the repository. Access to a private repository requires GitHub authentication.

```bash
git clone https://github.com/changjinwu94/cartpole_ros2.git ~/cartpole_ros2
cd ~/cartpole_ros2
```

For an existing checkout, start in its directory. On the development Pi this is `~/cartpole_repo_upload`. Select it explicitly so the trial runner and collector use the same source, configuration and logs.

```bash
export CARTPOLE_REPO="$PWD"
ls -l /dev/serial/by-id/
test -f .env || cp .env.example .env
nano .env
```

Set `ARDUINO_DEVICE` and `MOTOR_DEVICE` to their persistent device paths. Review the installed geometry, encoder direction and drive profile before using another rig. The supplied configuration enables motor communication. Keep a single Compose project in control of the serial devices.

## Build and check offline

```bash
docker compose config -q
docker compose build cartpole
python3 -m unittest discover -s experiments/cartpole_position_energy_v4 -q
```

The unit tests use fake hardware. Building the image and running these tests require no cart motion. For a live test, follow the [operator guide](docs/OPERATIONS.md). For comparison across trials, see [recordings and metrics](docs/DATA.md).

## Repository layout

| Path | Purpose |
|---|---|
| `cartpole_batch/` | Host launchers for one trial or repeated A/B pairs |
| `tools/cartpole-workflow.sh`, `tools/lqr/` | ROS-based preparation, centering and preflight |
| `experiments/cartpole_position_energy_v4/` | Standalone position controller, encoder processing, serial I/O and tests |
| `cartpole_hardware/` | ROS hardware node, configuration and launch file |
| `firmware/cartpole_sensors/` | Arduino encoder and endstop firmware |
| `logs/` | Local recordings, captured references and batch results |

Distributed under the [MIT License](LICENSE).
