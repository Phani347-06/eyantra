#!/usr/bin/env python3
"""Check recorded survivor holds without replaying or modifying a rosbag."""

import argparse
import json
import math
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from std_msgs.msg import Header
from whycode_interfaces.msg import MarkerArray


def check_bag(bag, survivors, coordinates):
    bag = Path(bag).resolve()
    files = list(bag.glob('*.db3')) if bag.is_dir() else [bag]
    if len(files) != 1 or not files[0].is_file():
        raise ValueError('Provide a bag directory containing one .db3 file, or that file')
    if len(survivors) != 4 or len(set(survivors)) != 4:
        raise ValueError('Provide four distinct survivor grid labels from the service log')

    targets = {}
    for label in survivors:
        point = coordinates[label]
        target = (float(point['x']), float(point['y']), 13.0)
        if not all(math.isfinite(value) for value in target):
            raise ValueError(f'Invalid coordinates for {label}')
        targets[label] = target

    connection = sqlite3.connect(files[0].as_uri() + '?mode=ro', uri=True)
    try:
        topics = {name: (topic_id, kind, encoding) for topic_id, name, kind, encoding
                  in connection.execute('SELECT id, name, type, serialization_format FROM topics')}
        marker_name = '/whycode_node/markers'
        heartbeat_name = '/controller_heartbeat'
        for name, kind in [(marker_name, 'whycode_interfaces/msg/MarkerArray'),
                           (heartbeat_name, 'std_msgs/msg/Header')]:
            if name not in topics or topics[name][1:] != (kind, 'cdr'):
                raise ValueError(f'Missing or incompatible topic: {name}')

        marker_id = topics[marker_name][0]
        heartbeat_id = topics[heartbeat_name][0]
        starts = {label: None for label in survivors}
        best_hold = {label: 0.0 for label in survivors}
        completed = {}
        mission_start = None
        previous_stamp = None
        previous_recorded = None
        marker_count = 0
        heartbeat_count = 0
        pre_start_markers = 0
        tracking_breaks = 0
        max_sample_gap = 0.0

        rows = connection.execute(
            'SELECT topic_id, timestamp, data FROM messages '
            'WHERE topic_id IN (?, ?) ORDER BY timestamp, id',
            (marker_id, heartbeat_id)
        )
        for topic_id, recorded_ns, payload in rows:
            recorded = recorded_ns * 1e-9
            if topic_id == heartbeat_id:
                heartbeat = deserialize_message(payload, Header)
                if heartbeat.frame_id != 'pico_controller_active':
                    continue
                heartbeat_count += 1
                if mission_start is None:
                    mission_start = recorded
                continue

            marker_count += 1
            message = deserialize_message(payload, MarkerArray)
            if mission_start is None:
                if message.markers:
                    pre_start_markers += 1
                continue

            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            interrupted = not message.markers
            if previous_stamp is not None:
                stamp_gap = stamp - previous_stamp
                record_gap = recorded - previous_recorded
                max_sample_gap = max(max_sample_gap, record_gap)
                interrupted |= stamp_gap < 0.0 or stamp_gap > 0.5 or record_gap > 0.5

            position = None
            if message.markers:
                # Use the first tracked marker, matching pico_server.
                point = message.markers[0].position.position
                position = (point.x, point.y, point.z)
                interrupted |= not all(math.isfinite(value) for value in position)
            if interrupted:
                starts = dict.fromkeys(survivors)
                tracking_breaks += 1
            previous_stamp, previous_recorded = stamp, recorded
            if position is None or not all(math.isfinite(value) for value in position):
                continue

            for label, target in targets.items():
                if label in completed:
                    continue
                inside = all(abs(actual - desired) <= 0.4 + 1e-9
                             for actual, desired in zip(position, target))
                if not inside:
                    starts[label] = None
                    continue
                if starts[label] is None:
                    starts[label] = (stamp, recorded)
                duration = stamp - starts[label][0]
                best_hold[label] = max(best_hold[label], duration)
                if duration >= 3.0:
                    completed[label] = {
                        'hold_seconds': round(duration, 6),
                        'hold_start_seconds_from_heartbeat': round(starts[label][1] - mission_start, 6),
                        'completion_seconds_from_heartbeat': round(recorded - mission_start, 6),
                    }

        elapsed = (max(item['completion_seconds_from_heartbeat'] for item in completed.values())
                   if len(completed) == 4 else None)
        return {
            'bag': str(files[0]),
            'all_four_holds_observed': len(completed) == 4,
            'fourth_hold_seconds_from_first_recorded_heartbeat': elapsed,
            'within_60_seconds': elapsed is not None and elapsed <= 60.0,
            'within_100_seconds': elapsed is not None and elapsed <= 100.0,
            'completed_holds': completed,
            'missing_holds': [label for label in survivors if label not in completed],
            'longest_observed_holds_seconds': {key: round(value, 6) for key, value in best_hold.items()},
            'marker_messages': marker_count,
            'controller_heartbeats': heartbeat_count,
            'marker_messages_before_first_heartbeat': pre_start_markers,
            'tracking_interruptions': tracking_breaks,
            'maximum_marker_record_gap_seconds': round(max_sample_gap, 6),
            'timing_note': 'Start recording before starting pico_server; a bag alone cannot prove this order.',
            'verification_note': 'Checks recorded samples, not motion between samples or an official score.',
        }
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='Stopped bag directory or .db3 file')
    parser.add_argument('--survivors', nargs=4, required=True, help='Detected labels from the service log')
    args = parser.parse_args()
    coordinates_file = Path(__file__).resolve().parents[1] / 'src/swift_pico/src/grid_whycode_coordinates.json'
    try:
        coordinates = json.loads(coordinates_file.read_text())
        result = check_bag(args.bag, args.survivors, coordinates)
    except (ValueError, KeyError, sqlite3.Error, OSError) as error:
        parser.exit(2, f'Cannot verify bag: {error}\n')
    print(json.dumps(result, indent=2))
    return 0 if result['all_four_holds_observed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
