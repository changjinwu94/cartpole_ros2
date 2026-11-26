"""Separate-process recorder: disk IO never runs in the motor control executor."""
import argparse
import csv
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=60)
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.duration <= 0:
        parser.error('duration must be positive')
    path = Path(parsed.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation avoids overwriting a previous experiment.
    with path.open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['receive_monotonic_ns', 'stream', 'payload_json'])
        rclpy.init(args=ros_args)
        node = Node('cartpole_recorder')
        subscriptions = []
        for topic in ('sensor_raw', 'telemetry', 'status'):
            def callback(message, name=topic):
                writer.writerow([time.monotonic_ns(), name, message.data])
            qos = 10 if topic == 'status' else qos_profile_sensor_data
            subscriptions.append(node.create_subscription(String, '/cartpole/' + topic, callback, qos))
        deadline = time.monotonic() + parsed.duration
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                stream.flush()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(str(path))
