# Running trials on robot01

Use a terminal on the Pi, including an SSH session from Windows. The launch scripts check for the hostname `robot01`. Run from the chosen checkout. For example, on the development Pi:

```bash
cd ~/cartpole_repo_upload
export CARTPOLE_REPO="$PWD"
```

## One swing

With the pole hanging freely, the track clear and the independent cutoff available, run:

```bash
unset CARTPOLE_BATCH
bash ./cartpole_batch/cartpole_final_push_trial.sh 0.3044 0.83 8
```

The arguments are the fraction of the remaining cart move on the final push, the normalized energy threshold that triggers it, and the trial timeout in seconds. The example requests 30.44% when the crossing energy reaches 0.83. The actual displacement depends on cart position at that crossing.

1. Type `prepare` to begin physical preparation. The ROS workflow homes, calibrates the hanging pole, centers the cart and checks its state.
2. The launcher captures a fresh reference, stops the normal ROS service to release both serial devices and starts a standalone container. The position runner commands the drive directly and enforces its own profile, travel, freshness and endstop checks.
3. After `PREFLIGHT PASS`, type `run now` to execute one trial. The reference expires 120 seconds after capture.
4. Check the execution summary for `motion_attempted`, `outcome`, and `stop_confirmed`. A normal exit restores the ROS service. If the driver reports an unconfirmed stop, use the independent cutoff and inspect the rig before restoring service.

## Collect paired trials

The following starts preparation and motion automatically. Five pairs run ten trials, alternating 0.3044 and 0.3043 with an energy trigger of 0.83 and an eight-second timeout. The collector accepts one to ten pairs and stops on failure.

```bash
bash "$CARTPOLE_REPO/cartpole_batch/cartpole_collect_pairs.sh" 5
```

The existing `bash ~/cartpole_batch/cartpole_collect_pairs.sh 5` command also works when that directory contains the matching launchers or points to the checkout's `cartpole_batch`. Set `CARTPOLE_REPO` in the shell that starts the job. This matters when comparing an upload checkout with the older working installation.

## Troubleshooting

| Symptom | Action |
|---|---|
| Missing serial device | Compare the host's /dev/serial/by-id entries with .env. Check for another running project holding the same devices. |
| Reference too old, or cart moved since capture | Repeat preparation and capture with the pole hanging and the cart centered. |
| Failed trial | Read the execution JSON and transcript for the first fault. A successful preflight has its own JSON and does not establish a successful swing. |
| Service stopped after failure | Confirm the cart has stopped, exit any standalone container, and run `docker compose up -d cartpole` from the same checkout. |
