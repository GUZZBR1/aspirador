import json
import os
import tempfile
import unittest
from types import SimpleNamespace

from robot_metrics.recorder import MetricsRecorder


def sample(events=None):
    return {
        'type': 'sample', 'schema_version': 2,
        'timestamp_utc': '2026-09-30T12:00:00+00:00',
        'sim_time_seconds': 1.0, 'state': None, 'battery_percent': None,
        'distance_m': None, 'distance_source': None, 'coverage_percent': None,
        'pose_frame_id': None, 'pose': None, 'velocity': None, 'cmd_vel': None,
        'events': events,
    }


def event_counts():
    return {
        'collision_proxy_lidar': 0, 'contact_confirmed': None,
        'stuck_events': 0, 'recovery_state_transitions': 0,
        'recovery_attempts_local': 0, 'recovery_attempts_nav2': None,
        'stuck_locations': [], 'return_seconds': [], 'failures': [],
    }


class RecorderSchemaTest(unittest.TestCase):
    def test_required_fields_and_nested_types(self):
        self.assertEqual(MetricsRecorder._validate_sample(sample()), [])
        row = sample(event_counts())
        self.assertEqual(MetricsRecorder._validate_sample(row), [])
        del row['battery_percent']
        row['events']['recovery_attempts_local'] = '0'
        row['events']['stuck_events'] = 1
        row['events']['stuck_locations'] = [{'x': 1, 'y': 2, 'frame_id': None}]
        errors = MetricsRecorder._validate_sample(row)
        self.assertIn('missing_battery_percent', errors)
        self.assertIn('invalid_events_recovery_attempts_local', errors)
        self.assertIn('invalid_events_stuck_locations', errors)

    def test_missing_measurements_remain_null(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SimpleNamespace(
                grid_pair=None, path=os.path.join(directory, 'run.jsonl'),
                summary_path=os.path.join(directory, 'summary.json'),
                samples_written=0, validation_errors=[], coverage_counts=None,
                events_observed=False, events=event_counts(), event_log=[],
                _write_json=MetricsRecorder._write_json,
            )
            MetricsRecorder._write_artifacts(recorder)
            with open(recorder.summary_path, encoding='utf-8') as stream:
                summary = json.load(stream)
            self.assertIsNone(summary['counts'])
            self.assertIsNone(summary['failure_count'])
            self.assertIsNone(summary['events'])
            self.assertEqual(summary['availability']['contact_confirmed'],
                             'unavailable_no_contact_sensor')
            self.assertEqual(summary['schema_version'], 2)

    def test_local_recovery_and_failure_counts(self):
        recorder = SimpleNamespace(
            odom=None, events_observed=False, event_log=[], events=event_counts(),
            validation_errors=[], start_time=None,
        )
        # A small clock object supplies the subtraction used by event logging.

        class ClockTime:
            def __sub__(self, other):
                return SimpleNamespace(nanoseconds=1000000000)

        recorder.get_clock = lambda: SimpleNamespace(now=ClockTime)
        for name in ('state:RECOVERY', 'stuck_detected', 'recovery_attempt:1',
                     'collision_proxy_lidar_clearance',
                     'return_to_dock_seconds:3.5', 'navigation_goal_failed',
                     'state:ERROR', 'dock_navigation_failed'):
            MetricsRecorder._event(recorder, SimpleNamespace(data=name))
        self.assertEqual(recorder.events['recovery_state_transitions'], 1)
        self.assertEqual(recorder.events['recovery_attempts_local'], 1)
        self.assertEqual(recorder.events['collision_proxy_lidar'], 1)
        self.assertIsNone(recorder.events['contact_confirmed'])
        self.assertEqual(recorder.events['stuck_locations'][0],
                         {'x': None, 'y': None, 'frame_id': None, 'source': None})
        self.assertEqual(recorder.events['return_seconds'], [3.5])
        self.assertEqual(recorder.events['failures'],
                         ['navigation_goal_failed', 'dock_navigation_failed'])
        self.assertEqual(MetricsRecorder._validate_sample(sample(recorder.events)), [])
        with tempfile.TemporaryDirectory() as directory:
            recorder.grid_pair = None
            recorder.path = os.path.join(directory, 'run.jsonl')
            recorder.summary_path = os.path.join(directory, 'summary.json')
            recorder.samples_written = 1
            recorder.coverage_counts = None
            recorder._write_json = MetricsRecorder._write_json
            MetricsRecorder._write_artifacts(recorder)
            with open(recorder.summary_path, encoding='utf-8') as stream:
                summary = json.load(stream)
            self.assertEqual(summary['counts']['collision_proxy_lidar'], 1)
            self.assertEqual(summary['counts']['recovery_attempts_local'], 1)
            self.assertIsNone(summary['counts']['contact_confirmed'])
            self.assertEqual(summary['counts']['completed_returns'], 1)
            self.assertEqual(summary['failure_count'], 2)
            self.assertEqual(summary['failure_cause_counts'],
                             {'navigation_goal_failed': 1, 'dock_navigation_failed': 1})
            self.assertEqual(summary['stuck_position_missing_count'], 1)
            self.assertEqual(summary['measurement_kind']['collision_proxy_lidar'],
                             'lidar_proximity_proxy')


if __name__ == '__main__':
    unittest.main()
