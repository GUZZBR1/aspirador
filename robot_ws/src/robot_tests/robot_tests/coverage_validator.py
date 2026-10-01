"""Validate saved exploration coverage artifacts and terminal outcome.

Run after an autonomous exploration session with:
    python -m robot_tests.coverage_validator path/to/run_*_summary.json
"""

import json
import math
import os
import sys

ROOM_BOUNDS = {
    'west': (-3.85, -0.18, -2.85, 2.85),
    'east_north': (0.18, 3.85, 0.42, 2.85),
    'east_south': (0.18, 3.85, -2.85, -0.42),
}


class CoverageReportError(ValueError):
    pass


def _load_json(path):
    try:
        with open(path, encoding='utf-8') as stream:
            return json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise CoverageReportError('cannot read JSON artifact %s: %s' % (path, exc)) from exc


def _room_for_position(x, y):
    return next((name for name, (xmin, xmax, ymin, ymax) in ROOM_BOUNDS.items()
                 if xmin < x < xmax and ymin < y < ymax), None)


def _map_to_world(x, y, metadata):
    if metadata is None:
        return x, y
    initial = metadata['observations']['initial_map_pose']
    spawn = metadata['initial_map']['world_spawn_sdf']
    map_yaw = math.atan2(2.0 * (initial['qw'] * initial['qz'] +
                                initial['qx'] * initial['qy']),
                         1.0 - 2.0 * (initial['qy'] ** 2 + initial['qz'] ** 2))
    delta = map_yaw - spawn['yaw']
    dx, dy = x - initial['x'], y - initial['y']
    return (spawn['x'] + math.cos(delta) * dx + math.sin(delta) * dy,
            spawn['y'] - math.sin(delta) * dx + math.cos(delta) * dy)


def _room_grid_counts(snapshot, metadata=None):
    grid = snapshot['grid']
    width, height = grid['width'], grid['height']
    resolution = grid['resolution']
    origin = grid['origin']
    qx, qy, qz, qw = (origin[key] for key in ('qx', 'qy', 'qz', 'qw'))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    counts = {name: {'free_cells': 0, 'visited_cells': 0} for name in ROOM_BOUNDS}
    for index, value in enumerate(snapshot['map_data']):
        if not 0 <= value < snapshot['free_threshold']:
            continue
        x, y = index % width, index // width
        lx, ly = (x + 0.5) * resolution, (y + 0.5) * resolution
        world_x = origin['x'] + math.cos(yaw) * lx - math.sin(yaw) * ly
        world_y = origin['y'] + math.sin(yaw) * lx + math.cos(yaw) * ly
        world_x, world_y = _map_to_world(world_x, world_y, metadata)
        room = _room_for_position(world_x, world_y)
        if room:
            counts[room]['free_cells'] += 1
            counts[room]['visited_cells'] += int(snapshot['coverage_data'][index] == 100)
    return counts


def _room_pose_evidence(trace_path, metadata=None):
    samples = {name: [] for name in ROOM_BOUNDS}
    try:
        with open(trace_path, encoding='utf-8') as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise CoverageReportError(
                        'invalid JSONL at line %d: %s' % (line_number, exc)) from exc
                if (row.get('type') != 'sample' or row.get('pose_frame_id') != 'map' or
                        not isinstance(row.get('pose'), dict)):
                    continue
                pose = row['pose']
                x, y = pose.get('x'), pose.get('y')
                if (type(x) not in (int, float) or type(y) not in (int, float) or
                        not math.isfinite(x) or not math.isfinite(y)):
                    continue
                world_x, world_y = _map_to_world(x, y, metadata)
                room = _room_for_position(world_x, world_y)
                if room:
                    samples[room].append((world_x, world_y))
    except OSError as exc:
        raise CoverageReportError('cannot read JSONL trace: %s' % exc) from exc
    return samples


def validate_coverage_report(summary_path, metadata_path=None):
    """Raise CoverageReportError unless a terminal run has reproducible coverage."""
    summary_path = os.path.abspath(summary_path)
    summary = _load_json(summary_path)
    directory = os.path.dirname(summary_path)

    if summary.get('schema_valid') is not True:
        raise CoverageReportError('summary schema_valid must be true')
    coverage_name = summary.get('coverage_snapshot')
    if not isinstance(coverage_name, str) or not coverage_name:
        raise CoverageReportError('summary has no paired coverage snapshot')
    snapshot = _load_json(os.path.join(directory, coverage_name))
    map_data = snapshot.get('map_data')
    coverage_data = snapshot.get('coverage_data')
    grid = snapshot.get('grid')
    if not isinstance(map_data, list) or not isinstance(coverage_data, list):
        raise CoverageReportError('snapshot must include map_data and coverage_data arrays')
    if not isinstance(grid, dict) or not grid.get('frame_id'):
        raise CoverageReportError('snapshot must include grid metadata and frame_id')
    if len(map_data) != len(coverage_data):
        raise CoverageReportError('map and coverage arrays have different lengths')
    if len(map_data) != grid.get('width', 0) * grid.get('height', 0):
        raise CoverageReportError('grid dimensions do not match cell arrays')

    threshold = snapshot.get('free_threshold')
    if not isinstance(threshold, (int, float)):
        raise CoverageReportError('snapshot has no numeric free_threshold')
    free_cells = 0
    visited_cells = 0
    for index, map_value in enumerate(map_data):
        coverage_value = coverage_data[index]
        if type(map_value) is not int or type(coverage_value) is not int:
            raise CoverageReportError('grid cells must be integer values')
        is_free = 0 <= map_value < threshold
        expected = (0, 100) if is_free else (-1,)
        if coverage_value not in expected:
            raise CoverageReportError('coverage cell %d violates map semantics' % index)
        free_cells += int(is_free)
        visited_cells += int(coverage_value == 100)
    if free_cells == 0:
        raise CoverageReportError('snapshot contains no known free cells')

    computed_percent = 100.0 * visited_cells / free_cells
    counts = summary.get('coverage')
    snapshot_counts = snapshot.get('counts')
    for label, value in (('summary', counts), ('snapshot', snapshot_counts)):
        if not isinstance(value, dict):
            raise CoverageReportError('%s has no coverage counts' % label)
        if value.get('known_free_cells') != free_cells:
            raise CoverageReportError('%s known_free_cells do not match grid' % label)
        if value.get('visited_cells') != visited_cells:
            raise CoverageReportError('%s visited_cells do not match grid' % label)
        percent = value.get('percent')
        if not isinstance(percent, (int, float)) or not math.isclose(
                percent, computed_percent, rel_tol=1e-7, abs_tol=1e-7):
            raise CoverageReportError('%s percent does not match grid counts' % label)

    jsonl_name = summary.get('jsonl')
    if not isinstance(jsonl_name, str) or not jsonl_name:
        raise CoverageReportError('summary has no JSONL trace')
    trace_path = os.path.join(directory, jsonl_name)
    if not os.path.isfile(trace_path):
        raise CoverageReportError('JSONL trace referenced by summary does not exist')
    events = summary.get('event_log')
    if not isinstance(events, list):
        raise CoverageReportError('summary has no event_log')
    terminal = next((entry.get('event', '') for entry in reversed(events)
                     if isinstance(entry, dict) and
                     (entry.get('event') == 'coverage_complete' or
                      str(entry.get('event', '')).startswith('coverage_incomplete:'))), None)
    if terminal is None:
        raise CoverageReportError('run has no coverage_complete or coverage_incomplete terminal event')

    reachability_event = next((entry.get('event', '') for entry in reversed(events)
                               if isinstance(entry, dict) and
                               str(entry.get('event', '')).startswith('coverage_reachability:')),
                              None)
    if reachability_event is None:
        raise CoverageReportError('terminal outcome has no connected-region reachability report')
    try:
        diagnostics = json.loads(reachability_event.split(':', 1)[1])
    except json.JSONDecodeError as exc:
        raise CoverageReportError('coverage reachability event has invalid diagnostics JSON') from exc
    components = diagnostics.get('known_free_components')
    unreachable = diagnostics.get('unreachable_components')
    if not isinstance(components, list) or not isinstance(unreachable, list):
        raise CoverageReportError('reachability report must include known and unreachable components')
    for component in components:
        if (not isinstance(component, dict) or
                not isinstance(component.get('component_id'), int) or
                type(component.get('reachable_from_robot')) is not bool):
            raise CoverageReportError('each known free component needs an id and reachability result')
        if (not component['reachable_from_robot'] and
                not component.get('unreachable_reason')):
            raise CoverageReportError('unreachable components need an explicit reason')
    for component in unreachable:
        if (not isinstance(component, dict) or
                not isinstance(component.get('component_id'), int) or
                not isinstance(component.get('reason'), str) or not component['reason']):
            raise CoverageReportError('every unreachable component must include an id and reason')
    reported_counts = diagnostics.get('coverage')
    reported_percent = (reported_counts.get('percent')
                        if isinstance(reported_counts, dict) else None)
    if (not isinstance(reported_counts, dict) or
            reported_counts.get('free_cells') != free_cells or
            reported_counts.get('visited_cells') != visited_cells or
            not isinstance(reported_percent, (int, float)) or
            not math.isfinite(reported_percent) or
            not math.isclose(reported_percent, computed_percent,
                             rel_tol=1e-7, abs_tol=1e-7)):
        raise CoverageReportError('reachability report coverage counts do not match saved grid')

    metadata = _load_json(metadata_path) if metadata_path is not None else None
    if metadata is not None:
        launch = metadata.get('launch')
        initial_map = metadata.get('initial_map')
        observations = metadata.get('observations')
        spawn = initial_map.get('world_spawn_sdf') if isinstance(initial_map, dict) else None
        initial_pose = (observations.get('initial_map_pose')
                        if isinstance(observations, dict) else None)
        aligned_fields = ('x', 'y', 'qx', 'qy', 'qz', 'qw')
        if (not isinstance(launch, list) or 'slam:=true' not in launch or
                any(arg.startswith('map:=') and arg != 'map:=' for arg in launch) or
                'autonomy_enabled:=true' not in launch or
                not isinstance(initial_map, dict) or
                initial_map.get('mode') != 'slam_from_scratch' or
                initial_map.get('static_map_path') != '' or
                initial_map.get('map_parameter_default') != '' or
                not isinstance(spawn, dict) or
                any(type(spawn.get(key)) not in (int, float) for key in ('x', 'y', 'yaw')) or
                not isinstance(initial_pose, dict) or
                any(type(initial_pose.get(key)) not in (int, float)
                    for key in aligned_fields)):
            raise CoverageReportError('run metadata does not prove blank-map SLAM and frame alignment')

    # Compare named room diagnostics to the saved grids and the robot's map-frame
    # trace. The boxes correspond to the three connected house interiors in casa.sdf.
    expected_rooms = _room_grid_counts(snapshot, metadata)
    room_diagnostics = diagnostics.get('rooms')
    if not isinstance(room_diagnostics, dict) or set(room_diagnostics) != set(ROOM_BOUNDS):
        raise CoverageReportError('terminal report must diagnose west, east_north, and east_south')
    for room_name, counts_for_room in expected_rooms.items():
        reported = room_diagnostics[room_name]
        if (not isinstance(reported, dict) or
                reported.get('free_cells') != counts_for_room['free_cells'] or
                reported.get('visited_cells') != counts_for_room['visited_cells']):
            raise CoverageReportError('terminal room counts do not match grid for %s' % room_name)
        if counts_for_room['visited_cells'] == 0:
            if (reported.get('reason') !=
                    'known_free_room_cells_disconnected_from_robot' or
                    reported.get('reachable_free_cells') != 0 or
                    not reported.get('unreachable_component_ids')):
                raise CoverageReportError(
                    'unvisited room %s is not proven unreachable from the robot' % room_name)
        elif reported.get('reason') is not None:
            raise CoverageReportError('visited room %s is incorrectly marked unreachable' % room_name)

    if terminal.startswith('coverage_incomplete:'):
        try:
            incomplete = json.loads(terminal.split(':', 1)[1])
        except json.JSONDecodeError as exc:
            raise CoverageReportError('coverage_incomplete event has invalid diagnostics JSON') from exc
        if incomplete != diagnostics:
            raise CoverageReportError('incomplete terminal details differ from reachability report')

    with open(trace_path, encoding='utf-8') as stream:
        first = next((json.loads(line) for line in stream if line.strip()), None)
    if not isinstance(first, dict) or first.get('state') != 'EXPLORING':
        raise CoverageReportError('trace does not show autonomous exploration at startup')

    pose_evidence = _room_pose_evidence(trace_path, metadata)
    unreachable_rooms = []
    for room_name, counts_for_room in expected_rooms.items():
        if counts_for_room['visited_cells'] == 0:
            unreachable_rooms.append(room_name)
            continue
        poses = pose_evidence[room_name]
        if len(poses) < 2 or max(
                math.hypot(a[0] - b[0], a[1] - b[1])
                for i, a in enumerate(poses) for b in poses[i + 1:]) < 0.10:
            raise CoverageReportError('trace does not prove traversal through room %s' % room_name)

    return {'outcome': 'coverage_complete' if terminal == 'coverage_complete'
            else 'coverage_incomplete', 'known_free_cells': free_cells,
            'visited_cells': visited_cells, 'percent': computed_percent,
            'unreachable_components': len(diagnostics['unreachable_components']),
            'rooms': {name: {'visited_cells': counts['visited_cells'],
                             'pose_samples': len(pose_evidence[name]),
                             'unreachable': name in unreachable_rooms}
                      for name, counts in expected_rooms.items()},
            'unreachable_rooms': unreachable_rooms}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) not in (1, 2):
        print('usage: python -m robot_tests.coverage_validator SUMMARY.json [RUN_METADATA.json]', file=sys.stderr)
        return 2
    try:
        result = validate_coverage_report(argv[0], argv[1] if len(argv) == 2 else None)
    except CoverageReportError as exc:
        print('coverage validation failed: %s' % exc, file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
