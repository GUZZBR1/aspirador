"""Run an end-to-end blank-map exploration and validate its saved evidence."""

import json
import os
import secrets
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from xml.etree import ElementTree
from glob import glob

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from robot_tests.coverage_validator import CoverageReportError, validate_coverage_report


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('coverage_acceptance_monitor')
        self.started = time.monotonic()
        self.terminal_event = None
        self.events = []
        self.first_state = None
        self.first_map = None
        self.initial_map_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, '/robot/event', self._event, 20)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot/state', self._state, latched)
        self.create_subscription(OccupancyGrid, '/map', self._map, latched)
        self.create_timer(0.1, self._capture_initial_map_pose)

    def _capture_initial_map_pose(self):
        if self.initial_map_pose is not None:
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link',
                                                        rclpy.time.Time())
        except Exception:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.initial_map_pose = {
            'x': translation.x, 'y': translation.y,
            'qx': rotation.x, 'qy': rotation.y,
            'qz': rotation.z, 'qw': rotation.w,
            'wall_seconds': time.monotonic() - self.started,
        }

    def _event(self, msg):
        event = msg.data
        observation = {'event': event, 'wall_seconds': time.monotonic() - self.started}
        self.events.append(observation)
        if event == 'coverage_complete' or event.startswith('coverage_incomplete:'):
            self.terminal_event = event

    def _state(self, msg):
        if self.first_state is None:
            self.first_state = {'state': msg.data,
                                'wall_seconds': time.monotonic() - self.started}

    def _map(self, msg):
        if self.first_map is None:
            self.first_map = {'frame_id': msg.header.frame_id,
                              'width': msg.info.width, 'height': msg.info.height,
                              'wall_seconds': time.monotonic() - self.started}


def _write_json(path, data):
    temporary = path + '.tmp'
    with open(temporary, 'w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
    os.replace(temporary, path)


def _stop_launch(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def run_coverage_acceptance(artifact_root, wall_timeout_seconds):
    if wall_timeout_seconds <= 0:
        raise ValueError('wall timeout must be positive')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    run_dir = os.path.abspath(os.path.join(artifact_root, 'issue5_' + stamp))
    os.makedirs(run_dir, exist_ok=False)
    domain_id = secrets.randbelow(190) + 20
    partition = 'aspirador-coverage-' + stamp.replace('T', '-').replace('Z', '')
    launch = ['ros2', 'launch', 'robot_simulation', 'mvp.launch.py',
              'headless:=true', 'slam:=true', 'navigation_enabled:=true',
              'autonomy_enabled:=true', 'metrics_enabled:=true']
    metadata_path = os.path.join(run_dir, 'run_metadata.json')
    log_path = os.path.join(run_dir, 'launch.log')
    description_share = get_package_share_directory('robot_description')
    sdf_path = os.path.join(description_share, 'worlds', 'casa.sdf')
    sdf_root = ElementTree.parse(sdf_path).getroot()
    robot_include = next(include for include in sdf_root.iter('include')
                         if include.findtext('name') == 'aspirador')
    spawn_pose = [float(value) for value in robot_include.findtext('pose').split()]
    metadata = {
        'schema_version': 1,
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'launch': launch,
        'ros_domain_id': domain_id,
        'gazebo_partition': partition,
        'initial_map': {'mode': 'slam_from_scratch', 'static_map_path': '',
                        'map_parameter_default': '',
                        'world_spawn_sdf': {'x': spawn_pose[0], 'y': spawn_pose[1],
                                            'yaw': spawn_pose[5],
                                            'source': sdf_path}},
        'observations': {'first_state': None, 'first_map': None,
                         'initial_map_pose': None, 'events': []},
        'terminal_event': None,
        'summary': None,
        'launch_returncode': None,
    }

    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)
    env['GZ_PARTITION'] = partition
    env['ASPIRADOR_RESULTS_DIR'] = run_dir
    resource_paths = [os.path.join(description_share, 'models'),
                      os.path.join(description_share, 'worlds')]
    resource_paths.extend(path for path in env.get('GZ_SIM_RESOURCE_PATH', '').split(':')
                          if path)
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join(resource_paths)
    monitor = None
    process = None
    timed_out = False
    interrupted = False
    terminal_event_drain_seconds = 2.0
    original_ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    original_gz_partition = os.environ.get('GZ_PARTITION')
    launch_log = open(log_path, 'w', encoding='utf-8')
    try:
        # The monitor and the launched simulator must use the same isolated
        # ROS domain; changing only Popen's environment leaves this node deaf.
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        os.environ['GZ_PARTITION'] = partition
        rclpy.init(args=[])
        monitor = ExplorationMonitor()
        process = subprocess.Popen(launch, env=env, stdout=launch_log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + wall_timeout_seconds
        while time.monotonic() < deadline and monitor.terminal_event is None:
            rclpy.spin_once(monitor, timeout_sec=0.25)
            if process.poll() is not None:
                break
        if monitor.terminal_event is not None:
            # Let the metrics recorder persist the same terminal event before
            # shutting down its node and writing the final summary.
            drain_deadline = time.monotonic() + terminal_event_drain_seconds
            while time.monotonic() < drain_deadline and process.poll() is None:
                rclpy.spin_once(monitor, timeout_sec=0.1)
        timed_out = monitor.terminal_event is None and process.poll() is None
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if process is not None:
            _stop_launch(process)
        launch_log.close()
        if monitor is not None:
            metadata['observations'] = {'first_state': monitor.first_state,
                                        'first_map': monitor.first_map,
                                        'initial_map_pose': monitor.initial_map_pose,
                                        'events': monitor.events}
            metadata['terminal_event'] = monitor.terminal_event
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if original_ros_domain_id is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = original_ros_domain_id
        if original_gz_partition is None:
            os.environ.pop('GZ_PARTITION', None)
        else:
            os.environ['GZ_PARTITION'] = original_gz_partition

    metadata['launch_returncode'] = process.returncode if process is not None else None
    metadata['terminal_event_drain_seconds'] = terminal_event_drain_seconds
    summaries = sorted(glob(os.path.join(run_dir, 'run_*_summary.json')))
    summary_path = summaries[-1] if summaries else None
    metadata['summary'] = os.path.basename(summary_path) if summary_path else None
    metadata['timed_out'] = timed_out
    metadata['interrupted'] = interrupted
    metadata['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    _write_json(metadata_path, metadata)
    if timed_out:
        raise CoverageReportError('exploration timed out before a terminal coverage event')
    if interrupted:
        raise CoverageReportError('exploration was interrupted before a terminal coverage event')
    if monitor is None or monitor.terminal_event is None:
        raise CoverageReportError('launch exited before a terminal coverage event')
    if summary_path is None:
        raise CoverageReportError('metrics recorder did not save a summary')
    result = validate_coverage_report(summary_path, metadata_path)
    result.update({'summary': summary_path, 'metadata': metadata_path,
                   'launch_log': log_path})
    return result


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', default=os.path.join(
        os.environ.get('ASPIRADOR_RESULTS_DIR', 'results'), 'issue5'))
    parser.add_argument('--wall-timeout-seconds', type=float, default=3600.0)
    args = parser.parse_args(argv)
    try:
        result = run_coverage_acceptance(args.artifact_dir, args.wall_timeout_seconds)
    except (CoverageReportError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print('coverage acceptance failed: %s' % exc, file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
