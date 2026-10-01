import json
import math
import os
from collections import Counter
from datetime import datetime, timedelta, timezone

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')
        self.declare_parameter('output_dir', os.environ.get('ASPIRADOR_RESULTS_DIR', 'results'))
        output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        os.makedirs(output_dir, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.path = os.path.join(output_dir, 'run_' + stamp + '.jsonl')
        self.summary_path = os.path.join(output_dir, 'run_' + stamp + '_summary.json')
        self.coverage_path = os.path.join(output_dir, 'run_' + stamp + '_coverage.json')
        self.file = open(self.path, 'a', encoding='utf-8', buffering=1)
        self.start_time = self.get_clock().now()
        self.odom = None
        self.battery = None
        self.coverage = None
        self.state = None
        self.cmd = None
        self.distance = None
        self.last_xy = None
        self.map_cache = {}
        self.pending_coverage = None
        self.grid_pair = None
        self.coverage_counts = None
        self.samples_written = 0
        self.validation_errors = []
        self.event_log = []
        self.events_observed = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.events = {'collision_proxy_lidar': 0, 'contact_confirmed': None,
                       'stuck_events': 0, 'recovery_state_transitions': 0,
                       'recovery_attempts_local': 0,
                       'recovery_attempts_nav2': None,
                       'stuck_locations': [], 'return_seconds': [], 'failures': []}
        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 20)
        self.create_subscription(BatteryState, '/battery_state', lambda m: setattr(self, 'battery', m.percentage), 10)
        self.create_subscription(Float32, '/coverage/percent', lambda m: setattr(self, 'coverage', m.data), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot/state', lambda m: setattr(self, 'state', m.data), latched)
        self.create_subscription(String, '/robot/event', self._event, 20)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(OccupancyGrid, '/map', self._map, latched)
        self.create_subscription(OccupancyGrid, '/coverage/grid', self._coverage_grid, latched)
        self.create_timer(1.0, self._write_sample)
        self.get_logger().info('Gravando telemetria em ' + self.path)

    def _odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_xy:
            self.distance += math.hypot(xy[0]-self.last_xy[0], xy[1]-self.last_xy[1])
        elif self.distance is None:
            self.distance = 0.0
        self.last_xy = xy

    def _map(self, msg):
        key = self._grid_key(msg)
        self.map_cache[key] = msg
        if len(self.map_cache) > 64:
            del self.map_cache[next(iter(self.map_cache))]
        if self.pending_coverage is not None and self._grid_key(self.pending_coverage) == key:
            pending = self.pending_coverage
            self.pending_coverage = None
            self._coverage_grid(pending)

    @staticmethod
    def _grid_key(grid):
        return (grid.header.frame_id, grid.header.stamp.sec, grid.header.stamp.nanosec)

    @staticmethod
    def _grid_info(grid):
        info = grid.info
        origin = info.origin
        return {'frame_id': grid.header.frame_id,
                'stamp': {'sec': grid.header.stamp.sec, 'nanosec': grid.header.stamp.nanosec},
                'width': info.width, 'height': info.height, 'resolution': info.resolution,
                'origin': {'x': origin.position.x, 'y': origin.position.y,
                           'z': origin.position.z, 'qx': origin.orientation.x,
                           'qy': origin.orientation.y, 'qz': origin.orientation.z,
                           'qw': origin.orientation.w}}

    def _coverage_grid(self, msg):
        mapped = self.map_cache.get(self._grid_key(msg))
        if mapped is None:
            self.pending_coverage = msg
            return
        if self._grid_info(mapped) != self._grid_info(msg):
            return
        if len(mapped.data) != mapped.info.width*mapped.info.height or len(msg.data) != len(mapped.data):
            return
        if any((value not in (-1, 0, 100) or
                ((0 <= mapped.data[i] < 25) != (value in (0, 100))))
               for i, value in enumerate(msg.data)):
            self.validation_errors.append('coverage_grid_inconsistent_with_map')
            return
        free = sum(value in (0, 100) for value in msg.data)
        visited = sum(value == 100 for value in msg.data)
        self.grid_pair = (mapped, msg)
        self.coverage_counts = {'known_free_cells': free, 'visited_cells': visited,
                                'percent': 100.0*visited/free if free else None}

    def _event(self, msg):
        event = msg.data
        self.events_observed = True
        location = None
        if self.odom is not None:
            pose, frame, source = self._position_pose()
            location = {'x': self._finite(pose.position.x),
                        'y': self._finite(pose.position.y),
                        'frame_id': frame, 'source': source}
        self.event_log.append({'event': event, 'sim_time_seconds':
                               (self.get_clock().now()-self.start_time).nanoseconds/1e9,
                               'pose': location})
        if event.startswith('collision_proxy'):
            self.events['collision_proxy_lidar'] += 1
        elif event == 'state:RECOVERY':
            self.events['recovery_state_transitions'] += 1
        elif event.startswith('recovery_attempt:'):
            self.events['recovery_attempts_local'] += 1
        elif event == 'stuck_detected':
            self.events['stuck_events'] += 1
            self.events['stuck_locations'].append(location or {
                'x': None, 'y': None, 'frame_id': None, 'source': None})
        elif event.startswith('return_to_dock_seconds:'):
            try:
                duration = float(event.split(':', 1)[1])
                if not math.isfinite(duration) or duration < 0:
                    raise ValueError('invalid duration')
                self.events['return_seconds'].append(duration)
            except ValueError:
                self.validation_errors.append('invalid_return_duration_event')
        elif event == 'state:ERROR':
            self.events['failures'].append(event)
        elif event.startswith(('coverage_incomplete:', 'dock_navigation_failed',
                               'dock_pose_unavailable', 'navigation_goal_failed')):
            if (event.startswith(('dock_navigation_failed', 'dock_pose_unavailable')) and
                    self.events['failures'] and self.events['failures'][-1] == 'state:ERROR'):
                self.events['failures'][-1] = event
            else:
                self.events['failures'].append(event)

    @staticmethod
    def _finite(value):
        return value if type(value) in (int, float) and math.isfinite(value) else None

    @classmethod
    def _valid_location(cls, location):
        if not isinstance(location, dict):
            return False
        if all(location.get(key) is None for key in ('x', 'y', 'frame_id', 'source')):
            return True
        return (cls._finite(location.get('x')) is not None and
                cls._finite(location.get('y')) is not None and
                isinstance(location.get('frame_id'), str) and bool(location['frame_id']) and
                location.get('source') in ('map_tf', 'odometry'))

    @classmethod
    def _validate_sample(cls, row):
        errors = []
        required = ('type', 'schema_version', 'timestamp_utc', 'sim_time_seconds',
                    'state', 'battery_percent', 'distance_m', 'distance_source',
                    'coverage_percent', 'pose_frame_id', 'pose', 'velocity', 'cmd_vel', 'events')
        errors.extend('missing_' + key for key in required if key not in row)
        if row.get('type') != 'sample' or row.get('schema_version') != 2:
            errors.append('invalid_sample_header')
        try:
            timestamp = datetime.fromisoformat(row['timestamp_utc'])
            if timestamp.utcoffset() != timedelta(0):
                raise ValueError('UTC timezone required')
        except (KeyError, TypeError, ValueError):
            errors.append('invalid_timestamp_utc')
        if row.get('state') is not None and not isinstance(row['state'], str):
            errors.append('invalid_state')
        if row.get('pose_frame_id') is not None and not isinstance(row['pose_frame_id'], str):
            errors.append('invalid_pose_frame_id')
        if cls._finite(row.get('sim_time_seconds')) is None or row['sim_time_seconds'] < 0:
            errors.append('invalid_sim_time_seconds')
        for key, lower, upper in (('battery_percent', 0, 100), ('coverage_percent', 0, 100)):
            value = row.get(key)
            if value is not None and (cls._finite(value) is None or not lower <= value <= upper):
                errors.append('invalid_' + key)
        if row.get('distance_m') is not None and (cls._finite(row['distance_m']) is None or row['distance_m'] < 0):
            errors.append('invalid_distance_m')
        if row.get('distance_source') not in (None, 'tracked_odometry'):
            errors.append('invalid_distance_source')
        if (row.get('distance_m') is None) != (row.get('distance_source') is None):
            errors.append('inconsistent_distance_source')
        for key, fields in (('pose', ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')),
                            ('velocity', ('linear_x', 'linear_y', 'angular_z')),
                            ('cmd_vel', ('linear_x', 'linear_y', 'angular_z'))):
            value = row.get(key)
            if value is not None and (not isinstance(value, dict) or
                                      any(cls._finite(value.get(field)) is None for field in fields)):
                errors.append('invalid_' + key)
        if row.get('pose') is not None and not isinstance(row.get('pose_frame_id'), str):
            errors.append('invalid_pose_frame_id')
        events = row.get('events')
        if events is not None:
            if not isinstance(events, dict):
                errors.append('invalid_events')
            else:
                for key in ('collision_proxy_lidar', 'stuck_events',
                            'recovery_state_transitions', 'recovery_attempts_local'):
                    if type(events.get(key)) is not int or events[key] < 0:
                        errors.append('invalid_events_' + key)
                locations = events.get('stuck_locations')
                if not isinstance(locations, list) or not all(
                        cls._valid_location(item) for item in locations):
                    errors.append('invalid_events_stuck_locations')
                durations = events.get('return_seconds')
                if not isinstance(durations, list) or not all(
                        cls._finite(item) is not None and item >= 0 for item in durations):
                    errors.append('invalid_events_return_seconds')
                failures = events.get('failures')
                if not isinstance(failures, list) or not all(
                        isinstance(item, str) and item for item in failures):
                    errors.append('invalid_events_failures')
                if (type(events.get('stuck_events')) is int and
                        isinstance(locations, list) and
                        events['stuck_events'] != len(locations)):
                    errors.append('inconsistent_stuck_locations')
                if (events.get('contact_confirmed') is not None or
                        events.get('recovery_attempts_nav2') is not None):
                    errors.append('invalid_unavailable_event_count')
        return errors

    def _write_sample(self):
        now = self.get_clock().now()
        row = {'type': 'sample', 'schema_version': 2,
               'timestamp_utc': datetime.now(timezone.utc).isoformat(),
               'sim_time_seconds': (now-self.start_time).nanoseconds/1e9,
               'state': self.state,
               'battery_percent': self._finite(None if self.battery is None else self.battery*100.0),
               'distance_m': self._finite(self.distance),
               'distance_source': 'tracked_odometry' if self._finite(self.distance) is not None else None,
               'coverage_percent': self._finite(self.coverage),
               'pose_frame_id': None, 'pose': None, 'velocity': None,
               'cmd_vel': None if self.cmd is None else {
                   'linear_x': self._finite(self.cmd.linear.x),
                   'linear_y': self._finite(self.cmd.linear.y),
                   'angular_z': self._finite(self.cmd.angular.z)},
               'events': dict(self.events) if self.events_observed else None}
        if self.odom:
            pose, frame, _source = self._position_pose()
            twist = self.odom.twist.twist
            row['pose_frame_id'] = frame
            row['pose'] = {'x': self._finite(pose.position.x), 'y': self._finite(pose.position.y),
                           'z': self._finite(pose.position.z),
                           'qx': self._finite(pose.orientation.x), 'qy': self._finite(pose.orientation.y),
                           'qz': self._finite(pose.orientation.z), 'qw': self._finite(pose.orientation.w)}
            row['velocity'] = {'linear_x': self._finite(twist.linear.x),
                               'linear_y': self._finite(twist.linear.y),
                               'angular_z': self._finite(twist.angular.z)}
        errors = self._validate_sample(row)
        if errors:
            self.validation_errors.extend(errors)
        row['schema_valid'] = not errors
        row['schema_errors'] = errors
        self.file.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')
        self.samples_written += 1
        if self.samples_written % 10 == 0:
            self._write_artifacts()

    def _write_artifacts(self):
        if self.grid_pair is not None:
            mapped, coverage = self.grid_pair
            payload = {'schema_version': 1, 'grid': self._grid_info(mapped),
                       'free_threshold': 25, 'map_data': list(mapped.data),
                       'coverage_data': list(coverage.data), 'counts': self.coverage_counts}
            self._write_json(self.coverage_path, payload)
        observed = self.events_observed
        failure_counts = Counter(event.split(':', 1)[0] for event in self.events['failures'])
        summary = {'schema_version': 2, 'jsonl': os.path.basename(self.path),
                   'coverage_snapshot': os.path.basename(self.coverage_path) if self.grid_pair else None,
                   'samples_written': self.samples_written,
                   'schema_valid': not self.validation_errors,
                   'validation_errors': sorted(set(self.validation_errors)),
                   'coverage': self.coverage_counts,
                   'events': dict(self.events) if observed else None,
                   'counts': {'collision_proxy_lidar': self.events['collision_proxy_lidar'],
                              'contact_confirmed': None,
                              'stuck_events': self.events['stuck_events'],
                              'recovery_attempts_local': self.events['recovery_attempts_local'],
                              'recovery_attempts_nav2': None,
                              'completed_returns': len(self.events['return_seconds'])} if observed else None,
                   'failure_count': len(self.events['failures']) if observed else None,
                   'failure_causes': sorted(set(self.events['failures'])) if observed else None,
                   'failure_cause_counts': dict(sorted(failure_counts.items())) if observed else None,
                   'stuck_position_missing_count': sum(
                       location['source'] is None for location in self.events['stuck_locations']) if observed else None,
                   'event_log': self.event_log,
                   'measurement_kind': {'collision_proxy_lidar': 'lidar_proximity_proxy',
                                        'contact_confirmed': 'unavailable_no_contact_sensor',
                                        'stuck_events': 'supervisor_progress_detection',
                                        'recovery_attempts_local': 'supervisor_event_count',
                                        'recovery_attempts_nav2': 'unavailable_no_nav2_event_subscription'},
                   'availability': {'contact_confirmed': 'unavailable_no_contact_sensor',
                                    'nav2_recoveries': 'unavailable_no_nav2_event_subscription',
                                    'local_recoveries': ('observed_event_stream' if observed else
                                                         'unavailable_no_events_observed'),
                                    'collision_proxy_lidar': ('observed_event_stream' if observed else
                                                              'unavailable_no_events_observed'),
                                    'return_duration': ('observed' if self.events['return_seconds'] else
                                                        'unavailable_no_completed_return_event'),
                                    'coverage': 'observed' if self.grid_pair else 'unavailable_no_paired_grids'}}
        self._write_json(self.summary_path, summary)

    @staticmethod
    def _write_json(path, payload):
        temporary = path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
        os.replace(temporary, path)

    def _position_pose(self):
        """Prefer the SLAM map pose; retain the source frame if TF is not ready."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t, r = transform.transform.translation, transform.transform.rotation
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
            pose.orientation = r
            return pose, 'map', 'map_tf'
        except Exception:
            return self.odom.pose.pose, self.odom.header.frame_id or 'odom', 'odometry'

    def destroy_node(self):
        if getattr(self, 'file', None) and not self.file.closed:
            self._write_artifacts()
            self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MetricsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
