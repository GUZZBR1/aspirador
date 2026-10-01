import json
import os
import tempfile
import unittest

from robot_tests.coverage_validator import (
    CoverageReportError,
    ROOM_BOUNDS,
    _map_to_world,
    _room_for_position,
    _room_grid_counts,
    validate_coverage_report,
)


class CoverageValidatorTest(unittest.TestCase):
    def test_aligns_arbitrary_map_origin_to_the_sdf_spawn(self):
        metadata = {
            'initial_map': {'world_spawn_sdf': {'x': -2.9, 'y': -2.15, 'yaw': 0.0}},
            'observations': {'initial_map_pose': {
                'x': 0.0, 'y': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}},
        }

        self.assertEqual(_map_to_world(0.0, 0.0, metadata), (-2.9, -2.15))
        x, y = _map_to_world(1.0, 0.0, metadata)
        self.assertAlmostEqual(x, -1.9)
        self.assertAlmostEqual(y, -2.15)

    def _fixture(self, missing_room=None, static_map_path=''):
        directory = tempfile.mkdtemp(prefix='coverage-validator-')
        width, height = 8, 6
        map_data = [0] * (width * height)
        coverage_data = [0] * (width * height)
        grid = {'frame_id': 'map', 'stamp': {'sec': 1, 'nanosec': 0},
                'width': width, 'height': height, 'resolution': 1.0,
                'origin': {'x': -4.0, 'y': -3.0, 'z': 0.0,
                           'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}}
        snapshot = {'schema_version': 1, 'grid': grid, 'free_threshold': 25,
                    'map_data': map_data, 'coverage_data': coverage_data,
                    'counts': None}
        cell_for_room = {}
        for index in range(width * height):
            x, y = index % width, index // width
            room = _room_for_position(-4.0 + x + 0.5, -3.0 + y + 0.5)
            if room and room not in cell_for_room:
                cell_for_room[room] = index
        for name, index in cell_for_room.items():
            if name != missing_room:
                coverage_data[index] = 100
        counts = _room_grid_counts(snapshot)
        free = len(map_data)
        visited = sum(value == 100 for value in coverage_data)
        percent = 100.0 * visited / free
        snapshot['counts'] = {'known_free_cells': free,
                              'visited_cells': visited, 'percent': percent}
        rooms = {}
        for name, values in counts.items():
            rooms[name] = dict(values, reachable_free_cells=values['free_cells'],
                               unreachable_component_ids=[],
                               coverage_percent=100.0 * values['visited_cells'] /
                               max(1, values['free_cells']),
                               reason=(None if values['visited_cells'] else
                                       'reachable_known_free_cells_unvisited_when_frontiers_exhausted'))
        diagnostics = {'coverage': {'free_cells': free, 'visited_cells': visited,
                                    'percent': percent},
                       'known_free_components': [{'component_id': 1,
                                                  'reachable_from_robot': True}],
                       'unreachable_components': [], 'rooms': rooms}
        reachability = 'coverage_reachability:' + json.dumps(diagnostics)
        terminal = ('coverage_complete' if missing_room is None else
                    'coverage_incomplete:' + json.dumps(diagnostics))
        events = [{'event': reachability}, {'event': terminal}]
        trace_name = 'trace.jsonl'
        samples = []
        centers = {'west': (-2.5, 0.5), 'east_north': (2.0, 1.5),
                   'east_south': (2.0, -1.5)}
        for room, (x, y) in centers.items():
            if room == missing_room:
                continue
            for dx in (0.0, 0.2):
                samples.append({'type': 'sample', 'state': 'EXPLORING',
                                'pose_frame_id': 'map',
                                'pose': {'x': x + dx, 'y': y}})
        with open(os.path.join(directory, trace_name), 'w', encoding='utf-8') as stream:
            for sample in samples:
                stream.write(json.dumps(sample) + '\n')
        snapshot_name = 'coverage.json'
        with open(os.path.join(directory, snapshot_name), 'w', encoding='utf-8') as stream:
            json.dump(snapshot, stream)
        summary_name = 'summary.json'
        with open(os.path.join(directory, summary_name), 'w', encoding='utf-8') as stream:
            json.dump({'schema_valid': True, 'coverage_snapshot': snapshot_name,
                       'jsonl': trace_name, 'coverage': snapshot['counts'],
                       'event_log': events}, stream)
        metadata_name = 'metadata.json'
        with open(os.path.join(directory, metadata_name), 'w', encoding='utf-8') as stream:
            json.dump({'launch': ['slam:=true', 'autonomy_enabled:=true'],
                       'initial_map': {'mode': 'slam_from_scratch',
                                       'static_map_path': static_map_path,
                                       'map_parameter_default': '',
                                       'world_spawn_sdf': {'x': -2.9, 'y': -2.15,
                                                           'yaw': 0.0}},
                       'observations': {'initial_map_pose':
                           {'x': -2.9, 'y': -2.15, 'qx': 0.0, 'qy': 0.0,
                            'qz': 0.0, 'qw': 1.0}}}, stream)
        return directory, summary_name, metadata_name

    def test_accepts_reproducible_coverage_through_three_rooms(self):
        directory, summary, metadata = self._fixture()
        result = validate_coverage_report(os.path.join(directory, summary),
                                         os.path.join(directory, metadata))
        self.assertEqual(set(result['rooms']), set(ROOM_BOUNDS))
        self.assertEqual(result['visited_cells'], 3)

    def test_rejects_run_without_visiting_a_room(self):
        directory, summary, metadata = self._fixture(missing_room='east_north')
        with self.assertRaisesRegex(CoverageReportError, 'not proven unreachable'):
            validate_coverage_report(os.path.join(directory, summary),
                                     os.path.join(directory, metadata))

    def test_rejects_static_map_input(self):
        directory, summary, metadata = self._fixture(static_map_path='/maps/old.yaml')
        with self.assertRaisesRegex(CoverageReportError, 'blank-map'):
            validate_coverage_report(os.path.join(directory, summary),
                                     os.path.join(directory, metadata))


if __name__ == '__main__':
    unittest.main()
