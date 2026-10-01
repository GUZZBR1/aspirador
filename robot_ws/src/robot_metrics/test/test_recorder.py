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
        'recovery_attempts_local': 0, 'navigation_goal_failures': 0,
        'recovery_attempts_nav2': None,
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
            path = os.path.join(directory, 'run.jsonl')
            open(path, 'w', encoding='utf-8').close()
            recorder = SimpleNamespace(
                grid_pair=None, path=path,
                summary_path=os.path.join(directory, 'summary.json'),
                samples_written=0, validation_errors=[], coverage_counts=None,
                events_observed=False, scan_seen=False,
                events=event_counts(), event_log=[],
                _write_json=MetricsRecorder._write_json,
            )
            MetricsRecorder._write_artifacts(recorder)
            with open(recorder.summary_path, encoding='utf-8') as stream:
                summary = json.load(stream)
            self.assertIsNone(summary['counts']['collision_proxy_lidar'])
            self.assertIsNone(summary['counts']['recovery_attempts_local'])
            self.assertIsNone(summary['failure_count'])
            self.assertIsNone(summary['events'])
            self.assertTrue(summary['schema_valid'])
            self.assertEqual(summary['offline_validation']['errors'], [])
            self.assertEqual(summary['availability']['contact_confirmed'],
                             'unavailable_no_contact_sensor')
            self.assertEqual(summary['schema_version'], 2)

    def test_local_recovery_and_failure_counts(self):
        recorder = SimpleNamespace(
            odom=None, events_observed=False, scan_seen=False,
            event_log=[], events=event_counts(),
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
        self.assertEqual(recorder.events['navigation_goal_failures'], 1)
        self.assertEqual(recorder.events['failures'], ['dock_navigation_failed'])
        self.assertEqual(MetricsRecorder._validate_sample(sample(recorder.events)), [])
        with tempfile.TemporaryDirectory() as directory:
            recorder.grid_pair = None
            recorder.path = os.path.join(directory, 'run.jsonl')
            open(recorder.path, 'w', encoding='utf-8').close()
            recorder.summary_path = os.path.join(directory, 'summary.json')
            recorder.samples_written = 0
            recorder.coverage_counts = None
            recorder._write_json = MetricsRecorder._write_json
            MetricsRecorder._write_artifacts(recorder)
            with open(recorder.summary_path, encoding='utf-8') as stream:
                summary = json.load(stream)
            self.assertIsNone(summary['counts']['collision_proxy_lidar'])
            self.assertEqual(summary['availability']['collision_proxy_lidar'],
                             'unavailable_no_scan_observed')
            self.assertEqual(summary['counts']['recovery_attempts_local'], 1)
            self.assertEqual(summary['counts']['navigation_goal_failures'], 1)
            self.assertIsNone(summary['counts']['contact_confirmed'])
            self.assertEqual(summary['counts']['completed_returns'], 1)
            self.assertEqual(summary['failure_count'], 1)
            self.assertEqual(summary['failure_cause_counts'],
                             {'dock_navigation_failed': 1})
            self.assertEqual(summary['stuck_position_missing_count'], 1)
            self.assertEqual(summary['measurement_kind']['collision_proxy_lidar'],
                             'lidar_proximity_proxy')

    def test_scan_observation_enables_zero_proxy_count(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'run.jsonl')
            open(path, 'w', encoding='utf-8').close()
            recorder = SimpleNamespace(
                grid_pair=None, path=path,
                summary_path=os.path.join(directory, 'summary.json'),
                samples_written=0, validation_errors=[], coverage_counts=None,
                events_observed=False, scan_seen=False,
                events=event_counts(), event_log=[],
                _write_json=MetricsRecorder._write_json,
            )
            MetricsRecorder._write_artifacts(recorder)
            with open(recorder.summary_path, encoding='utf-8') as stream:
                before = json.load(stream)
            self.assertIsNone(before['counts']['collision_proxy_lidar'])
            MetricsRecorder._scan(recorder, SimpleNamespace())
            MetricsRecorder._write_artifacts(recorder)
            with open(recorder.summary_path, encoding='utf-8') as stream:
                after = json.load(stream)
            self.assertEqual(after['counts']['collision_proxy_lidar'], 0)
            self.assertEqual(after['availability']['collision_proxy_lidar'], 'observed_scan')

    def test_offline_jsonl_detects_truncation_and_schema_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'run.jsonl')
            row = sample()
            row['schema_valid'] = True
            row['schema_errors'] = []
            with open(path, 'w', encoding='utf-8') as stream:
                stream.write(json.dumps(row) + '\n')
            self.assertEqual(MetricsRecorder._validate_saved_jsonl(path, 1), (1, []))
            with open(path, 'a', encoding='utf-8') as stream:
                stream.write('{"type":"sample"')
            count, errors = MetricsRecorder._validate_saved_jsonl(path, 2)
            self.assertEqual(count, 2)
            self.assertIn('jsonl_line_2_missing_newline', errors)
            self.assertIn('jsonl_line_2_invalid_json', errors)
            with open(path, 'w', encoding='utf-8') as stream:
                row['battery_percent'] = 'unknown'
                stream.write(json.dumps(row) + '\n')
            _, errors = MetricsRecorder._validate_saved_jsonl(path, 1)
            self.assertIn('jsonl_line_1_invalid_battery_percent', errors)
            self.assertIn('jsonl_line_1_schema_result_mismatch', errors)
            self.assertIn('jsonl_sample_count_mismatch',
                          MetricsRecorder._validate_saved_jsonl(path, 2)[1])

    def test_offline_coverage_recomputes_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'coverage.json')
            counts = {'known_free_cells': 3, 'visited_cells': 1,
                      'percent': 100.0/3}
            snapshot = {'schema_version': 1,
                        'grid': {'width': 2, 'height': 2, 'resolution': 0.05},
                        'free_threshold': 25,
                        'map_data': [0, 100, -1, 0],
                        'coverage_data': [100, -1, -1, 0],
                        'counts': {'known_free_cells': 2, 'visited_cells': 1,
                                   'percent': 50.0}}
            with open(path, 'w', encoding='utf-8') as stream:
                json.dump(snapshot, stream)
            self.assertEqual(MetricsRecorder._validate_saved_coverage(
                path, snapshot['counts']), [])
            self.assertIn('coverage_summary_counts_mismatch',
                          MetricsRecorder._validate_saved_coverage(path, counts))
            snapshot['coverage_data'][1] = 0
            with open(path, 'w', encoding='utf-8') as stream:
                json.dump(snapshot, stream)
            self.assertIn('coverage_snapshot_map_mask_mismatch',
                          MetricsRecorder._validate_saved_coverage(path, snapshot['counts']))

    def test_invalid_previous_snapshot_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'run.jsonl')
            coverage_path = os.path.join(directory, 'coverage.json')
            open(path, 'w', encoding='utf-8').close()
            with open(coverage_path, 'w', encoding='utf-8') as stream:
                stream.write('{"incomplete":')
            counts = {'known_free_cells': 2, 'visited_cells': 1, 'percent': 50.0}
            recorder = SimpleNamespace(
                grid_pair=object(), coverage_path=coverage_path,
                saved_coverage_counts=counts, coverage_counts=counts,
                path=path, summary_path=os.path.join(directory, 'summary.json'),
                samples_written=0, validation_errors=[], events_observed=False,
                scan_seen=False, events=event_counts(), event_log=[],
                _write_json=MetricsRecorder._write_json,
            )
            MetricsRecorder._write_artifacts(recorder)
            with open(coverage_path, encoding='utf-8') as stream:
                self.assertEqual(stream.read(), '{"incomplete":')
            with open(recorder.summary_path, encoding='utf-8') as stream:
                summary = json.load(stream)
            self.assertFalse(summary['schema_valid'])
            self.assertEqual(summary['availability']['coverage'], 'invalid_snapshot')
            self.assertIn('coverage_snapshot_unavailable_or_invalid_json',
                          summary['offline_validation']['errors'])

    def test_invalid_jsonl_does_not_replace_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'run.jsonl')
            coverage_path = os.path.join(directory, 'coverage.json')
            with open(path, 'w', encoding='utf-8') as stream:
                stream.write('{"incomplete":')
            counts = {'known_free_cells': 1, 'visited_cells': 1, 'percent': 100.0}
            original = {'schema_version': 1,
                        'grid': {'width': 1, 'height': 1, 'resolution': 0.05},
                        'free_threshold': 25, 'map_data': [0],
                        'coverage_data': [100], 'counts': counts}
            with open(coverage_path, 'w', encoding='utf-8') as stream:
                json.dump(original, stream)
            recorder = SimpleNamespace(
                grid_pair=object(), coverage_path=coverage_path,
                saved_coverage_counts=counts, coverage_counts=counts,
                path=path, summary_path=os.path.join(directory, 'summary.json'),
                samples_written=1, validation_errors=[], events_observed=False,
                scan_seen=False, events=event_counts(), event_log=[],
                _write_json=MetricsRecorder._write_json,
            )
            MetricsRecorder._write_artifacts(recorder)
            with open(coverage_path, encoding='utf-8') as stream:
                self.assertEqual(json.load(stream), original)
            with open(recorder.summary_path, encoding='utf-8') as stream:
                summary = json.load(stream)
            self.assertFalse(summary['schema_valid'])
            self.assertIn('jsonl_line_1_missing_newline',
                          summary['offline_validation']['errors'])


if __name__ == '__main__':
    unittest.main()
