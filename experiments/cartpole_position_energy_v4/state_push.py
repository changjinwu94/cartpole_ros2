"""Position-target selection from the Isaac Lab probe, using Pi crossing events."""
import argparse
import json
import math
from pathlib import Path


def state_push_target(current, endpoint, pole_rate, direction):
    if not math.isfinite(pole_rate) or direction not in (-1, 1):
        raise ValueError("Invalid crossing direction or pole rate")
    if direction * (endpoint - current) <= 0:
        return None, 0.0
    fraction = 1.0
    speed = -pole_rate
    if direction < 0 and 20.0 <= speed <= 21.05:
        blend = max(0.0, min(1.0, (speed - 20.12) / 0.82))
        fraction = 0.8 - 0.4 * blend
    return current + round(fraction * (endpoint - current)), fraction


def main():
    parser = argparse.ArgumentParser(description="Compare position decisions offline")
    parser.add_argument("recording", type=Path)
    args = parser.parse_args()
    recording = json.loads(args.recording.read_text())
    decisions = []
    for event in recording["events"]:
        if event["kind"] != "energy_decision":
            continue
        target, fraction = state_push_target(
            event["current_counts"], event["endpoint_counts"],
            event["pole_rate_rad_s"], event["direction"],
        )
        decisions.append(dict(
            stamp_ns=event["host_monotonic_ns"], original=event["target_counts"],
            state_push=target, fraction=fraction,
        ))
    if not decisions:
        parser.error("Recording contains no energy_decision events")
    print(json.dumps(decisions, indent=2))


if __name__ == "__main__":
    main()
