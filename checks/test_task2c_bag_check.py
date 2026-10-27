#!/usr/bin/env python3
"""Exercise the bag checker with serialized ROS messages, without ROS nodes."""

import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

from rclpy.serialization import serialize_message
from std_msgs.msg import Header
from whycode_interfaces.msg import Marker, MarkerArray

from check_task2c_bag import check_bag


class BagCheckTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='khojo_bag_check_')
        self.addCleanup(self.directory.cleanup)
        self.bag = Path(self.directory.name) / 'test.db3'
        root = Path(__file__).resolve().parents[1]
        self.coordinates = json.loads((root / 'src/swift_pico/src/grid_whycode_coordinates.json').read_text())
        self.survivors = ['H7', 'H5', 'E4', 'D7']

    def write_bag(self, fault=None):
        connection = sqlite3.connect(self.bag)
        connection.executescript(
            'CREATE TABLE topics (id INTEGER, name TEXT, type TEXT, serialization_format TEXT);'
            'CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);'
        )
        connection.executemany('INSERT INTO topics VALUES (?, ?, ?, ?)', [
            (1, '/whycode_node/markers', 'whycode_interfaces/msg/MarkerArray', 'cdr'),
            (2, '/controller_heartbeat', 'std_msgs/msg/Header', 'cdr'),
        ])

        def append(topic, seconds, message):
            stamp_ns = 1792690000000000000 + round(seconds * 1e9)
            connection.execute('INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)',
                               (topic, stamp_ns, serialize_message(message)))

        def marker(seconds, label, sample=-1):
            message = MarkerArray()
            stamp_ns = 1792690000000000000 + round(seconds * 1e9)
            if label == 'H7' and fault == 'clock_jump' and sample >= 16:
                stamp_ns -= 2000000000
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
            item = Marker()
            position = item.position.position
            point = self.coordinates[label]
            position.x, position.y, position.z = float(point['x']), float(point['y']), 13.0
            message.markers = [item]
            if label == 'H7' and sample >= 0:
                if fault == 'boundary':
                    position.x += 0.4
                    position.y -= 0.4
                    position.z += 0.4
                if sample == 16:
                    if fault == 'outside': position.x += 0.401
                    if fault == 'altitude': position.z += 0.401
                    if fault == 'empty': message.markers = []
                    if fault == 'nan': position.x = float('nan')
            append(1, seconds, message)

        marker(0.0, 'H7')
        if fault != 'no_heartbeat':
            heartbeat = Header()
            heartbeat.frame_id = 'pico_controller_active'
            append(2, 0.5, heartbeat)
        for index, label in enumerate(self.survivors):
            for sample in range(32):
                seconds = 2.0 + index * 6.0 + sample * 0.1
                if label == 'H7' and fault == 'gap' and sample >= 16:
                    seconds += 0.6
                marker(seconds, label, sample)
        connection.commit()
        connection.close()

    def test_valid_holds_and_read_only(self):
        self.write_bag()
        before = hashlib.sha256(self.bag.read_bytes()).hexdigest()
        result = check_bag(self.bag, self.survivors, self.coordinates)
        self.assertTrue(result['all_four_holds_observed'])
        self.assertTrue(result['within_60_seconds'])
        self.assertEqual(result['marker_messages_before_first_heartbeat'], 1)
        self.assertEqual(before, hashlib.sha256(self.bag.read_bytes()).hexdigest())

    def test_inclusive_box_boundary(self):
        self.write_bag('boundary')
        self.assertTrue(check_bag(self.bag, self.survivors, self.coordinates)['all_four_holds_observed'])

    def test_interruptions_do_not_count_as_continuous_holds(self):
        for fault in ['outside', 'altitude', 'empty', 'nan', 'gap', 'clock_jump', 'no_heartbeat']:
            with self.subTest(fault=fault):
                if self.bag.exists(): self.bag.unlink()
                self.write_bag(fault)
                result = check_bag(self.bag, self.survivors, self.coordinates)
                self.assertFalse(result['all_four_holds_observed'])
                self.assertIn('H7', result['missing_holds'])


if __name__ == '__main__':
    unittest.main()
