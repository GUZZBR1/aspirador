"""Ground-truth mobility probes for the threshold and bed fixtures."""

import json
import math
import os
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


THRESHOLD_START = (-0.65, 0.0)
THRESHOLD_ROBOT_REAR_EXTENT_M = 0.173
THRESHOLD_EXIT_MARGIN_M = 0.03
BED_START = (1.35, 1.75)
BED_CENTER = (2.65, 1.75)
BED_CENTER_GOAL = (2.95, 1.75)
BED_EXIT = (4.05, 1.75)
BED_ENTRY_PLANE_X = 1.825
BED_EXIT_PLANE_X = 3.475
BED_ROUTE_HALF_WIDTH = 0.25
ROBOT_ENVELOPE_HEIGHT_M = 0.14  # Rounded up from the 0.133 m lidar mount.


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def roll_pitch_of(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def bed_geometry_from_sdf():
    """Read the installed fixture; the report must reflect its actual dimensions."""
    path = os.path.join(get_package_share_directory('robot_description'),
                        'worlds', 'bed_test.sdf')
    world = ET.parse(path).getroot()
    bed = world.find(".//model[@name='bed']")
    if bed is None:
        raise ValueError('bed model missing from bed_test.sdf')
    bed_pose = [float(v) for v in bed.findtext('pose', '0 0 0 0 0 0').split()]
    mattress = bed.find(".//collision[@name='mattress']")
    if mattress is None:
        raise ValueError('mattress collision missing from bed_test.sdf')
    mattress_pose = [float(v) for v in mattress.findtext('pose').split()]
    size = [float(v) for v in mattress.findtext('geometry/box/size').split()]
    if any(abs(angle) > 1e-9 for angle in bed_pose[3:] + mattress_pose[3:]):
        raise ValueError('bed clearance calculation requires axis-aligned geometry')
    legs = []
    for leg in bed.findall(".//collision"):
        if not leg.attrib.get('name', '').startswith('leg'):
            continue
        xyz = [float(v) for v in leg.findtext('pose').split()[:3]]
        legs.append((bed_pose[0] + xyz[0], bed_pose[1] + xyz[1]))
    if len(legs) != 4:
        raise ValueError('bed fixture must have four leg collisions')
    return {
        'underside_m': bed_pose[2] + mattress_pose[2] - size[2] / 2.0,
        'x_min': bed_pose[0] + mattress_pose[0] - size[0] / 2.0,
        'x_max': bed_pose[0] + mattress_pose[0] + size[0] / 2.0,
        'leg_centers_world': legs,
        'source': path,
    }


def threshold_geometry_from_sdf():
    path = os.path.join(get_package_share_directory('robot_description'),
                        'worlds', 'threshold_test.sdf')
    world = ET.parse(path).getroot()
    collision = world.find(".//model[@name='soleira_12mm']//collision[@name='threshold_collision']")
    if collision is None:
        raise ValueError('threshold collision missing from threshold_test.sdf')
    size = [float(v) for v in collision.findtext('geometry/box/size').split()]
    return {'height_m': size[2], 'length_m': size[0], 'width_m': size[1], 'source': path}


class PhysicsRunner(Node):
    def __init__(self):
        super().__init__('physics_runner')
        self.declare_parameter('scenario', 'threshold')
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('startup_timeout', 90.0)
        self.declare_parameter('test_timeout', 120.0)
        self.scenario = str(self.get_parameter('scenario').value)
        if self.scenario not in ('threshold', 'bed'):
            raise ValueError('scenario must be threshold or bed')
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        self.test_timeout = float(self.get_parameter('test_timeout').value)
        self.fixture = threshold_geometry_from_sdf() if self.scenario == 'threshold' else bed_geometry_from_sdf()
        self.wall_start = time.monotonic()
        self.attempt_wall_start = None
        self.attempt_sim_start = None
        self.finished = False
        self.success = False
        self.reason = None
        self.pose = None
        self.start_pose = None
        self.last_logged_sim = None
        self.samples = []
        self.distance_m = 0.0
        self.max_height_m = None
        self.min_clearance_m = None
        self.last_progress_pose = None
        self.last_progress_sim = None
        self.threshold_settled_since = None
        self.map_msg = None
        self.map_anchor = None
        self.stage = 0
        self.pending = None
        self.pending_kind = None
        self.goal_handle = None
        self.gates = {'entry': False, 'center': False, 'exit': False}
        self.paths = []
        self.leg_map_observations = []
        self.startup_diagnostics = {
            'pose_available': False,
            'sim_clock_available': False,
            'map_available': self.scenario != 'bed',
            'map_width': None,
            'map_height': None,
            'map_data_length': None,
            'planner_ready': self.scenario != 'bed',
            'navigator_ready': self.scenario != 'bed',
            'planner_active': self.scenario != 'bed',
            'navigator_active': self.scenario != 'bed',
            'map_to_base_transform_available': self.scenario != 'bed',
        }
        self.cmd = Twist()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      '/initialpose', 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(TFMessage, '/physics/pose_info', self._pose_cb, 10)
        if self.scenario == 'bed':
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
            self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.lifecycle_clients = {
                'planner_active': self.create_client(GetState, '/planner_server/get_state'),
                'navigator_active': self.create_client(GetState, '/bt_navigator/get_state'),
            }
            self.lifecycle_futures = {key: None for key in self.lifecycle_clients}
            self.lifecycle_active = {key: False for key in self.lifecycle_clients}
        self.create_timer(0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _sim_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _pose_cb(self, msg):
        transforms = msg.transforms
        match = next((t for t in transforms
                      if t.child_frame_id in ('aspirador', 'model::aspirador')), None)
        if match is None:
            match = next((t for t in transforms
                          if t.child_frame_id == 'aspirador::base_link'), None)
        if match is None:
            match = next((t for t in transforms if t.child_frame_id == 'aspirador'), None)
        if match is None:
            match = next((t for t in transforms
                          if t.child_frame_id.endswith('/base_link') or
                          t.child_frame_id.endswith('::base_link') or
                          t.child_frame_id == 'base_link'), None)
        if match is None:
            return
        p, q = match.transform.translation, match.transform.rotation
        roll, pitch = roll_pitch_of(q)
        self.pose = {'x': p.x, 'y': p.y, 'z': p.z,
                     'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w,
                     'yaw': yaw_of(q), 'roll': roll, 'pitch': pitch}

    def _map_cb(self, msg):
        self.map_msg = msg

    def _publish_bed_initial_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = BED_START[0]
        pose.pose.pose.position.y = BED_START[1]
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.05
        pose.pose.covariance[7] = 0.05
        pose.pose.covariance[35] = 0.02
        self.initial_pose_pub.publish(pose)

    def _sample(self, sim_now):
        if self.pose is None or sim_now <= 0:
            return
        if self.last_logged_sim is not None and sim_now - self.last_logged_sim < 0.095:
            return
        current = dict(self.pose)
        current['sim_time_s'] = round(sim_now, 3)
        if self.samples:
            previous = self.samples[-1]
            self.distance_m += math.hypot(current['x'] - previous['x'], current['y'] - previous['y'])
        self.samples.append(current)
        self.last_logged_sim = sim_now
        self.max_height_m = max(self.max_height_m or current['z'], current['z'])
        if self.scenario == 'bed' and self.attempt_sim_start is not None:
            self._sample_bed(current)

    def _sample_bed(self, pose):
        x, y = pose['x'], pose['y']
        if abs(y - BED_CENTER[1]) <= BED_ROUTE_HALF_WIDTH:
            self.gates['entry'] |= x >= BED_ENTRY_PLANE_X + 0.05
            self.gates['center'] |= x >= BED_CENTER[0] - 0.20
            self.gates['exit'] |= x >= BED_EXIT_PLANE_X + 0.23
        if self.fixture['x_min'] <= x <= self.fixture['x_max']:
            if abs(y - BED_CENTER[1]) > BED_ROUTE_HALF_WIDTH:
                self._finish(False, 'detour_around_bed')
                return
            # Account conservatively for body pitch and roll while under the bed.
            effective_top = (pose['z'] + ROBOT_ENVELOPE_HEIGHT_M
                             + 0.225 * abs(math.sin(pose['pitch']))
                             + 0.178 * abs(math.sin(pose['roll'])))
            clearance = self.fixture['underside_m'] - effective_top
            self.min_clearance_m = min(self.min_clearance_m, clearance) if self.min_clearance_m is not None else clearance
            if clearance <= 0:
                self._finish(False, 'insufficient_vertical_clearance')

    def _tick(self):
        if self.finished:
            return
        sim_now = self._sim_seconds()
        self._sample(sim_now)
        if self.finished:
            return
        if self.attempt_wall_start is None:
            if time.monotonic() - self.wall_start > self.startup_timeout:
                self._finish(False, self._startup_failure_reason())
                return
            self.startup_diagnostics['pose_available'] = self.pose is not None
            self.startup_diagnostics['sim_clock_available'] = sim_now > 0
            if self.scenario == 'bed':
                grid = self.map_msg
                if grid is not None:
                    self.startup_diagnostics.update({
                        'map_width': grid.info.width,
                        'map_height': grid.info.height,
                        'map_data_length': len(grid.data),
                    })
                    self.startup_diagnostics['map_available'] = (
                        grid.info.width > 0 and grid.info.height > 0 and
                        len(grid.data) == grid.info.width * grid.info.height)
                if self.startup_diagnostics['map_available']:
                    self._publish_bed_initial_pose()
                self.startup_diagnostics['planner_ready'] = self.planner.server_is_ready()
                self.startup_diagnostics['navigator_ready'] = self.navigator.server_is_ready()
                for key, client in self.lifecycle_clients.items():
                    future = self.lifecycle_futures[key]
                    if future is None and client.service_is_ready():
                        self.lifecycle_futures[key] = client.call_async(GetState.Request())
                    elif future is not None and future.done():
                        try:
                            state = future.result().current_state.id
                            self.lifecycle_active[key] = state == State.PRIMARY_STATE_ACTIVE
                        except Exception:
                            self.lifecycle_active[key] = False
                        self.lifecycle_futures[key] = None
                self.startup_diagnostics.update(self.lifecycle_active)
                if self.startup_diagnostics['map_available']:
                    try:
                        self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                        self.startup_diagnostics['map_to_base_transform_available'] = True
                    except Exception:
                        self.startup_diagnostics['map_to_base_transform_available'] = False
            if self.pose is None or sim_now <= 0:
                return
            if self.scenario == 'threshold':
                self._start_threshold(sim_now)
            else:
                self._start_bed(sim_now)
            return
        if time.monotonic() - self.attempt_wall_start > self.test_timeout:
            self._finish(False, 'test_timeout')
            return
        if self.scenario == 'threshold':
            self._tick_threshold(sim_now)
        else:
            self._tick_bed(sim_now)

    def _begin(self, sim_now):
        self.attempt_wall_start = time.monotonic()
        self.attempt_sim_start = sim_now
        self.start_pose = dict(self.pose)
        self.samples = []
        self.distance_m = 0.0
        self.max_height_m = None
        self.last_logged_sim = None
        self.last_progress_pose = dict(self.pose)
        self.last_progress_sim = sim_now

    def _start_threshold(self, sim_now):
        if (abs(self.pose['x'] - THRESHOLD_START[0]) > 0.08 or
                abs(self.pose['y'] - THRESHOLD_START[1]) > 0.05 or
                abs(self.pose['yaw']) > 0.12):
            self._finish(False, 'invalid_initial_alignment')
            return
        self._begin(sim_now)
        self.get_logger().info('Beginning direct 12 mm threshold probe')

    def _tick_threshold(self, sim_now):
        if abs(self.pose['y']) > 0.12 or abs(self.pose['yaw']) > 0.30:
            self._finish(False, 'left_frontal_corridor')
            return
        exit_plane = (self.fixture['length_m'] / 2.0 +
                      THRESHOLD_ROBOT_REAR_EXTENT_M + THRESHOLD_EXIT_MARGIN_M)
        rear_clear = self.pose['x'] >= exit_plane
        settled = self.pose['z'] <= 0.005 and abs(self.pose['pitch']) <= 0.03
        if rear_clear and settled:
            if self.threshold_settled_since is None:
                self.threshold_settled_since = sim_now
            elif sim_now - self.threshold_settled_since >= 0.5:
                self._finish(True, 'full_robot_cleared_threshold_and_settled')
                return
        else:
            self.threshold_settled_since = None
        if self.pose['x'] >= self.last_progress_pose['x'] + 0.02:
            self.last_progress_pose = dict(self.pose)
            self.last_progress_sim = sim_now
        if sim_now - self.last_progress_sim > 5.0:
            self._finish(False, 'blocked_without_forward_progress')
            return
        command = Twist()
        command.linear.x = 0.12
        self.cmd_pub.publish(command)

    def _start_bed(self, sim_now):
        if self.fixture['underside_m'] <= ROBOT_ENVELOPE_HEIGHT_M:
            self._finish(False, 'fixture_has_insufficient_nominal_clearance')
            return
        if (abs(self.pose['x'] - BED_START[0]) > 0.08 or
                abs(self.pose['y'] - BED_START[1]) > 0.08 or
                abs(self.pose['yaw']) > 0.12):
            self._finish(False, 'invalid_initial_alignment')
            return
        if self.map_msg is None or not self.map_msg.data:
            return
        if not self.planner.server_is_ready() or not self.navigator.server_is_ready():
            return
        if not all(self.lifecycle_active.values()):
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        p, q = transform.transform.translation, transform.transform.rotation
        self.map_anchor = {'world_x': self.pose['x'], 'world_y': self.pose['y'],
                           'map_x': p.x, 'map_y': p.y,
                           'yaw_delta': yaw_of(q) - self.pose['yaw']}
        self._begin(sim_now)
        self._request_path(BED_CENTER_GOAL)
        self.get_logger().info('Beginning bed passage through center and exit gates')

    def _world_to_map(self, x, y):
        a = self.map_anchor
        dx, dy = x - a['world_x'], y - a['world_y']
        c, s = math.cos(a['yaw_delta']), math.sin(a['yaw_delta'])
        return a['map_x'] + c * dx - s * dy, a['map_y'] + s * dx + c * dy

    def _map_to_world(self, x, y):
        a = self.map_anchor
        dx, dy = x - a['map_x'], y - a['map_y']
        c, s = math.cos(a['yaw_delta']), math.sin(a['yaw_delta'])
        return a['world_x'] + c * dx + s * dy, a['world_y'] - s * dx + c * dy

    def _goal_pose(self, world_xy):
        x, y = self._world_to_map(*world_xy)
        yaw = self.map_anchor['yaw_delta']
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _request_path(self, target):
        goal = ComputePathToPose.Goal()
        goal.goal = self._goal_pose(target)
        goal.use_start = False
        self.pending = self.planner.send_goal_async(goal)
        self.pending_kind = 'plan_accept'

    def _request_navigation(self, target):
        goal = NavigateToPose.Goal()
        goal.pose = self._goal_pose(target)
        self.pending = self.navigator.send_goal_async(goal)
        self.pending_kind = 'nav_accept'

    def _tick_bed(self, sim_now):
        if self.pending is not None and self.pending.done():
            try:
                result = self.pending.result()
                if self.pending_kind in ('plan_accept', 'nav_accept'):
                    if not result.accepted:
                        self._finish(False, self.pending_kind + '_rejected')
                        return
                    self.goal_handle = result
                    self.pending = result.get_result_async()
                    self.pending_kind = ('plan_result' if self.pending_kind == 'plan_accept' else 'nav_result')
                    return
                if self.pending_kind == 'plan_result':
                    if result.status != GoalStatus.STATUS_SUCCEEDED:
                        self._finish(False, 'planner_failed')
                        return
                    if not self._record_and_check_path(result.result.path):
                        self._finish(False, 'planner_path_avoids_underbed_corridor_or_hits_leg')
                        return
                    self._request_navigation(BED_CENTER_GOAL if self.stage == 0 else BED_EXIT)
                    return
                if result.status != GoalStatus.STATUS_SUCCEEDED:
                    self._finish(False, 'navigation_failed_stage_' + str(self.stage))
                    return
                if self.stage == 0:
                    if not (self.gates['entry'] and self.gates['center']):
                        self._finish(False, 'center_goal_did_not_cross_underbed_gates')
                        return
                    self.stage = 1
                    self._request_path(BED_EXIT)
                else:
                    self.leg_map_observations = self._inspect_leg_map()
                    if not self.gates['exit']:
                        self._finish(False, 'exit_goal_did_not_clear_bed')
                    elif not all(row['occupied'] for row in self.leg_map_observations):
                        self._finish(False, 'bed_legs_not_all_represented_in_map')
                    else:
                        self._finish(True, 'passed_under_bed_using_known_leg_map')
                return
            except Exception as exc:
                self._finish(False, 'action_exception: ' + str(exc))
                return
        if self.pose is not None and self.last_progress_pose is not None:
            movement = math.hypot(self.pose['x'] - self.last_progress_pose['x'],
                                  self.pose['y'] - self.last_progress_pose['y'])
            if movement >= 0.03:
                self.last_progress_pose = dict(self.pose)
                self.last_progress_sim = sim_now
            elif sim_now - self.last_progress_sim > 20.0:
                self._finish(False, 'blocked_without_position_progress')

    def _record_and_check_path(self, path):
        xy = [self._map_to_world(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if len(xy) < 2:
            self.paths.append({'stage': self.stage, 'points_world': [], 'valid': False,
                               'reason': 'fewer_than_two_points'})
            return False
        sampled = []
        for (x0, y0), (x1, y1) in zip(xy, xy[1:]):
            steps = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / 0.02))
            sampled.extend((x0 + (x1 - x0) * i / steps,
                            y0 + (y1 - y0) * i / steps) for i in range(steps))
        sampled.append(xy[-1])
        target_range = (2.60, 3.10) if self.stage == 0 else (2.90, 3.35)
        gate_seen = False
        rejection = None
        for x, y in sampled:
            if self.fixture['x_min'] <= x <= self.fixture['x_max']:
                if abs(y - BED_CENTER[1]) > BED_ROUTE_HALF_WIDTH:
                    rejection = 'outside_underbed_corridor'
                    break
            if target_range[0] <= x <= target_range[1] and abs(y - BED_CENTER[1]) <= BED_ROUTE_HALF_WIDTH:
                gate_seen = True
            if any(math.hypot(x - lx, y - ly) < 0.28
                   for lx, ly in self.fixture['leg_centers_world']):
                rejection = 'path_intersects_leg_clearance'
                break
        if rejection is None and not gate_seen:
            rejection = 'missed_stage_gate'
        self.paths.append({'stage': self.stage, 'points_world':
                           [{'x': round(x, 3), 'y': round(y, 3)} for x, y in xy],
                           'valid': rejection is None, 'reason': rejection})
        return rejection is None

    def _map_cell(self, x, y):
        grid = self.map_msg
        origin = grid.info.origin.position
        q = grid.info.origin.orientation
        yaw = yaw_of(q)
        dx, dy = x - origin.x, y - origin.y
        gx = int(math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / grid.info.resolution))
        gy = int(math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / grid.info.resolution))
        if not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height):
            return -1
        return int(grid.data[gy * grid.info.width + gx])

    def _inspect_leg_map(self):
        observations = []
        for wx, wy in self.fixture['leg_centers_world']:
            mx, my = self._world_to_map(wx, wy)
            values = [self._map_cell(mx + dx, my + dy)
                      for dx in (-0.05, 0.0, 0.05)
                      for dy in (-0.05, 0.0, 0.05)]
            observations.append({'world_x': wx, 'world_y': wy,
                                 'map_x': mx, 'map_y': my,
                                 'max_occupancy': max(values),
                                 'occupied': any(value >= 50 for value in values)})
        return observations

    def _finish(self, passed, reason):
        if self.finished:
            return
        self.finished = True
        self.success = bool(passed)
        self.reason = reason
        if self.scenario == 'bed' and self.map_anchor is not None and self.map_msg is not None:
            self.leg_map_observations = self._inspect_leg_map()
        self.cmd_pub.publish(Twist())
        if self.goal_handle is not None and not passed:
            self.goal_handle.cancel_goal_async()
        os.makedirs(self.output_dir, exist_ok=True)
        elapsed_sim = (self._sim_seconds() - self.attempt_sim_start
                       if self.attempt_sim_start is not None else None)
        report = {
            'suite': 'physics_' + self.scenario,
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'passed': self.success,
            'reason': reason,
            'attempted': self.attempt_sim_start is not None,
            'distance_m': round(self.distance_m, 4),
            'duration_sim_s': round(elapsed_sim, 3) if elapsed_sim is not None else None,
            'duration_wall_s': round(time.monotonic() - (self.attempt_wall_start or self.wall_start), 3),
            'start_pose_world': self.start_pose,
            'end_pose_world': self.pose,
            'max_base_height_m': self.max_height_m,
            'pose_samples_world': self.samples,
            'fixture': self.fixture,
        }
        if self.scenario == 'threshold':
            report['exit_plane_world_x_m'] = (self.fixture['length_m'] / 2.0 +
                                               THRESHOLD_ROBOT_REAR_EXTENT_M +
                                               THRESHOLD_EXIT_MARGIN_M)
            report['initial_distance_to_near_face_m'] = (
                -self.fixture['length_m'] / 2.0 - self.start_pose['x']
                if self.start_pose is not None else None)
            report['maximum_absolute_lateral_offset_m'] = max(
                (abs(sample['y']) for sample in self.samples), default=None)
            report['max_tested_passed_height_m'] = self.fixture['height_m'] if passed else None
            report['max_tested_passed_height_scope'] = 'single tested height; not an absolute robot limit'
            filename = 'threshold_report.json'
        else:
            report['gates_passed'] = self.gates
            report['planned_paths'] = self.paths
            report['leg_map_observations'] = self.leg_map_observations
            report['leg_map_source'] = 'prebuilt_static_fixture_map'
            report['online_leg_detection_validated'] = False
            report['planner_leg_clearance_validated'] = all(path['valid'] for path in self.paths)
            report['robot_envelope_height_m'] = ROBOT_ENVELOPE_HEIGHT_M
            report['nominal_clearance_m'] = self.fixture['underside_m'] - ROBOT_ENVELOPE_HEIGHT_M
            report['minimum_observed_clearance_m'] = self.min_clearance_m
            report['collision_or_blockage_method'] = 'ground-truth progress stall and vertical envelope; no contact sensor'
            filename = 'bed_clearance_report.json'
        report['startup_diagnostics'] = self.startup_diagnostics
        path = os.path.join(self.output_dir, filename)
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
        self.get_logger().info('%s: %s (%s)' % (filename, 'PASS' if passed else 'FAIL', reason))

    def _startup_failure_reason(self):
        if not self.startup_diagnostics['pose_available']:
            return 'gazebo_ground_truth_pose_unavailable'
        if not self.startup_diagnostics['sim_clock_available']:
            return 'simulation_clock_unavailable'
        if not self.startup_diagnostics['map_available']:
            return 'slam_map_unavailable_or_malformed'
        if not self.startup_diagnostics['map_to_base_transform_available']:
            return 'map_to_base_transform_unavailable'
        if not (self.startup_diagnostics['planner_ready'] and
                self.startup_diagnostics['navigator_ready']):
            return 'nav2_action_servers_unavailable'
        return 'startup_timeout'


def main():
    rclpy.init()
    node = PhysicsRunner()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if not node.finished:
            node._finish(False, 'interrupted')
    passed = node.success
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
