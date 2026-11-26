# Cartpole position-target batch trials

ROS 2 Humble on robot01 with a Faulhaber drive and Arduino sensor board.
The batch compares 0.3044 and 0.3043 at a 0.83 energy trigger. Each trial
prepares the hardware, captures a position reference, checks preflight, and
runs the bounded swing-up. Launching the batch initiates physical motion.

## Install on a commissioned host

Use ~/cartpole_ros2 as the checkout path. Configure .env from .env.example
with both actual serial device paths. On the existing robot use the known
working .env. Build with docker compose build cartpole. Review the hardware
configuration in cartpole_hardware/config/cartpole.yaml before commissioning.

On a fresh host, link the bundled launchers with
`ln -sT "$HOME/cartpole_ros2/cartpole_batch" "$HOME/cartpole_batch"`.
An existing installation may already have this directory. Keep the known-working
copy or back it up before replacing it with the link.

Run `bash ~/cartpole_batch/cartpole_collect_pairs.sh 5` on robot01.
Results are written under logs/final_push_pairs.*. The runner stops at upright
entry. Run offline runner tests with
`python3 -m unittest discover -s experiments/cartpole_position_energy_v4`.
