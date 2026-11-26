"""Summarize a commissioning recording with no ROS or external dependencies."""
import argparse
import csv
import json
import math
from collections import Counter


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), mean=sum(values)/len(values),
                p95=values[max(0, math.ceil(.95*len(values))-1)], max=values[-1])


def analyze(path):
    counts = Counter()
    intervals, loops, queries, ages = [], [], [], []
    transitions = []
    previous = None
    board_intervals = []
    gaps = 0
    faults = set()
    modes = set()
    with open(path, newline='') as stream:
        for row in csv.DictReader(stream):
            payload = json.loads(row['payload_json'])
            topic = row['stream']
            counts[topic] += 1
            if topic == 'sensor_raw':
                if previous:
                    delta = (payload['sequence']-previous['sequence']) & 0xffffffff
                    if 0 < delta < 0x80000000:
                        gaps += delta-1
                        board_intervals.append(((payload['board_us']-previous['board_us']) & 0xffffffff)/1000)
                        intervals.append((payload['host_monotonic_ns']-previous['host_monotonic_ns'])/1e6)
                    for side in ('left_blocked', 'right_blocked'):
                        if payload[side] != previous[side]:
                            transitions.append(dict(sensor=side, blocked=payload[side],
                                                    host_monotonic_ns=payload['host_monotonic_ns']))
                previous = payload
            elif topic == 'telemetry':
                loops.append(payload['loop_dt_s']*1000)
                if payload.get('motor_available'):
                    queries.append(payload['motor_roundtrip_s']*1000)
                ages.append(payload['sensor_age_s']*1000)
                modes.add(payload['mode'])
            if payload.get('fault'):
                faults.add(payload['fault'])
    return dict(records=dict(counts), observed_sensor_sequence_gaps=gaps,
                board_interval_ms=stats(board_intervals), host_arrival_interval_ms=stats(intervals),
                loop_interval_ms=stats(loops), motor_POS_GN_roundtrip_ms=stats(queries),
                sensor_age_at_telemetry_ms=stats(ages), switch_transitions=transitions,
                modes=sorted(modes), faults=sorted(faults),
                limitation='Sequence gaps can include recorder DDS losses; arrival/age metrics are not physical end-to-end stopping latency.')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file')
    parsed = parser.parse_args(args)
    print(json.dumps(analyze(parsed.csv_file), indent=2))


if __name__ == '__main__':
    main()
