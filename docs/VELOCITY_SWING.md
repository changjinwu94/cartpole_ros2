# Velocity control for the final-push swing test

From the checkout on robot01, select `--velocity` on the single-trial or paired launcher. With no mode flag the launcher uses the position experiment. The velocity runner is copied into the running ROS 2 Humble container at invocation. The existing image and motor configuration are sufficient.

## Check and run

These commands exercise the code offline, including signed driver commands, crossing detection, travel limits, stopping and collection.

```bash
cd ~/cartpole_repo_upload
export CARTPOLE_REPO="$PWD"
python3 -m unittest discover -s experiments/cartpole_position_energy_v4 -q
python3 -m unittest discover -s tests -q
python3 experiments/cartpole_position_energy_v4/run_velocity.py --speed 0.08 --acceleration 1.0
```

For one physical trial, start with the pole hanging down, the rail clear and the independent cutoff available. The first prompt starts homing, pole calibration and centering. The launcher captures the current reference and checks for a disabled, stationary cart, a settled pole and fresh feedback. At `PREFLIGHT PASS`, entering `run now` starts the velocity swing. Execution rechecks the reference and hardware before enabling. The reference expires 120 s after capture.

```bash
unset CARTPOLE_BATCH
bash ./cartpole_batch/cartpole_final_push_trial.sh --velocity 0.3044 0.83 8 0.08 1.0
```

| Argument | Meaning in this example |
| --- | --- |
| `0.3044` | Final stroke covers 30.44% of the distance remaining to the selected endpoint. |
| `0.83` | Apply that reduction on the first eligible crossing at or above 83% of the ideal mechanical energy needed to reach upright. |
| `8` | Swing timeout in seconds after enable, maximum 20 s. |
| `0.08` | Requested speed cap in m/s, maximum 0.10 m/s. |
| `1.0` | Requested acceleration slew in m/s², maximum 1.0 m/s². The braking envelope can reduce the command sooner. |

For a live read-only preflight against an already-prepared, running node with a fresh reference, use `bash ./cartpole_batch/cartpole_final_push_trial.sh --velocity --check-only`. This invocation skips homing and leaves the drive disabled. An ordinary single trial performs its own preparation and preflight.

To collect five velocity pairs, run the following after reviewing a single velocity recording. This command starts physical preparation and runs up to ten trials. Each pair uses 0.3044 then 0.3043 with trigger 0.83 and timeout 8 s. A failed trial or unconfirmed stop ends the series.

```bash
bash ./cartpole_batch/cartpole_collect_pairs.sh --velocity 5 0.08 1.0
```

## Motor commands and recordings

The velocity client reads the calibrated pole angle, pole rate, cart position and measured speed from the normal 50 Hz ROS telemetry. The existing phase policy chooses the next direction at a downward crossing. Its nominal endpoints are +15.12 mm and −12.37 mm relative to the start. These endpoints set the distance budget for each stroke. Every fresh sample produces a signed speed request on `/cartpole/cmd_velocity`. The ROS hardware node converts m/s to motor RPM and sends the Faulhaber `V` command. The position runner uses the drive's `LA` and `M` commands and has its own measured response.

The command tapers as the remaining stroke shortens. The runner also uses a 5 mm reserve inside the intersection of the 50–100 mm cart window and ±22 mm about the start. Its braking calculation uses 0.10 m/s² or the trial slew, whichever is lower. Measured cart speed above the configured 1500 motor RPM equivalent also aborts the trial. The hardware node applies its configured velocity, acceleration, motor RPM, rail and endstop limits and the 0.25 s command watchdog. On the first observed sample within ±12° of upright, the runner requests zero and calls the normal feedback-confirmed stop service. Upright balancing would require a separate capture controller.

Each velocity invocation writes `logs/velocity_final_push_*.json`. A velocity pair series writes `logs/velocity_final_push_pairs.*/results.csv` and copies the trial recordings beside it. A `velocity_command` event contains the current request (`requested_m_s`), the node output reported in the triggering telemetry sample (`applied_m_s`), and the encoder-derived cart speed in that sample (`measured_m_s`). Feedback precedes the new request. Each `energy_decision` includes the crossing direction, energy, selected stroke endpoint and fraction. Inspect `summary.outcome`, `summary.closest_upright_deg` and `summary.stop_confirmed`. A completed trial can finish at the time limit or on the return after the final push. Only `upright_entry` indicates entry into the upright window.

The commissioning defaults are caps. The paired position recordings on robot01 contained peak measured cart speeds around 0.27 m/s. A velocity trial limited to 0.10 m/s has a different attainable stroke profile, and its swing-up performance needs measurement.
