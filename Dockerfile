FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-serial python3-colcon-common-extensions ros-humble-launch-ros \
    ros-humble-sensor-msgs ros-humble-std-srvs ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY cartpole_hardware src/cartpole_hardware
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cartpole_hardware", "cartpole.launch.py", "config:=/config/cartpole.yaml"]
