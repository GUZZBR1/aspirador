import json
import math
import os
import time
import hashlib
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def _matches_expected(expect, outcome):
    if expect == 'success':
        return outcome == 'goal_reached'
    if expect == 'failure':
        return outcome in ('invalid_goal', 'nav2_failure')
    return outcome == expect


class MobilityRunner(Node):
    def __init__(self):
        super().__init__('mobility_runner')
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('goal_timeout', 100.0)
        self.declare_parameter('setup_timeout', 90.0)
        self.declare_parameter('startup_timeout', 120.0)
        self.declare_parameter('scenario_limit', 8)
        self.declare_parameter('scenario_name', '')
        self.declare_parameter('run_control_tests', True)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.timeout = float(self.get_parameter('goal_timeout').value)
        self.setup_timeout = float(self.get_parameter('setup_timeout').value)
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        scenarios_path = os.path.join(get_package_share_directory('robot_tests'), 'scenarios', 'mobility.json')
        with open(scenarios_path, encoding='utf-8') as stream:
            self.cases = json.load(stream)
        self.world_spawn_pose = self._robot_spawn_pose()
        scenario_name = str(self.get_parameter('scenario_name').value)
        if scenario_name:
            self.cases = [case for case in self.cases if case['name'] == scenario_name]
            if not self.cases:
                raise ValueError('Unknown mobility scenario: ' + scenario_name)
        else:
            self.cases = self.cases[:max(0, int(self.get_parameter('scenario_limit').value))]
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_pub = self.create_publisher(String, '/robot/control', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(self, 'odom', m), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_pose = None
        self.world_pose_received_at = None
        self.create_subscription(TFMessage, '/model/aspirador/pose', self._world_pose_cb, 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 self._collision_state_cb, 10)
        self.cmd = Twist()
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.map_seen = False
        self.map_msg = None
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 1)
        self.event_sub = self.create_subscription(String, '/robot/event', lambda _: None, 10)
        self.robot_state = ''
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot/state', lambda m: setattr(self, 'robot_state', m.data), state_qos)
        self.pause_requested_at = None
        self.last_pause_request = 0.0
        self.idle_clear_since = None
        self.index = 0
        self.future = None
        self.goal_handle = None
        self.waiting_for_result = False
        self.goal_sent_at = None
        self.stage = 'setup'
        self.setup_targets = []
        self.setup_target_index = 0
        self.setup_retry_limit = 2
        self.case_record = None
        self.pending_outcome = None
        self.cancel_future = None
        self.settle_started = None
        self.stopped_since = None
        self.last_route_sample = 0.0
        self.results = []
        self.finished = False
        self.started_wall = time.monotonic()
        self.control_index = 0
        if scenario_name or not bool(self.get_parameter('run_control_tests').value):
            self.control_index = 3
        self.control_start = None
        self.control_pose = None
        self.create_timer(0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _map_cb(self, msg):
        self.map_seen = True
        self.map_msg = msg

    @staticmethod
    def _yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _yaw_error(actual, target):
        return math.atan2(math.sin(actual - target), math.cos(actual - target))

    def _setup_yaw(self, case):
        """Face the next setup segment (or the trial goal) before starting a probe."""
        target = self.setup_targets[self.setup_target_index]
        if self.setup_target_index + 1 < len(self.setup_targets):
            following = self.setup_targets[self.setup_target_index + 1]
        else:
            following = case['goal_map']
        dx, dy = following[0] - target[0], following[1] - target[1]
        if math.hypot(dx, dy) < 1e-6:
            return 0.0
        return math.atan2(dy, dx)

    @classmethod
    def _pose_dict(cls, position, orientation):
        return {'x': position.x, 'y': position.y, 'z': position.z,
                'yaw': cls._yaw(orientation)}

    def _world_pose_cb(self, msg):
        # The Gazebo pose bridge is separate from /tf: these are simulator poses,
        # not localization transforms. Never select an unnamed pose by index.
        model_pose = None
        base_pose = None
        for transform in msg.transforms:
            child = transform.child_frame_id.strip('/')
            is_model = (child == 'aspirador' or child.endswith('/aspirador') or
                        child.endswith('::aspirador'))
            is_base_link = (child.endswith('aspirador/base_link') or
                            child.endswith('aspirador::base_link'))
            if is_model:
                model_pose = transform
            elif is_base_link:
                base_pose = transform
        selected = model_pose or base_pose
        if selected is not None:
            self.world_pose = self._pose_dict(selected.transform.translation,
                                              selected.transform.rotation)
            self.world_pose_received_at = time.monotonic()

    def _plan_cb(self, msg):
        if self.case_record is not None and self.stage == 'goal' and self.future is not None:
            self.case_record['planned_route_map'] = [
                self._pose_dict(p.pose.position, p.pose.orientation)
                for p in msg.poses]
            self.case_record['plan_update_count'] = self.case_record.get('plan_update_count', 0) + 1

    def _collision_state_cb(self, msg):
        if self.case_record is None or self.stage != 'goal' or int(msg.action_type) == 0:
            return
        events = self.case_record.setdefault('collision_monitor_events', {})
        key = '%s:%s' % (int(msg.action_type), msg.polygon_name or 'unnamed')
        event = events.setdefault(key, {'action_type': int(msg.action_type),
                                         'polygon_name': msg.polygon_name,
                                         'count': 0, 'first_sim_time_seconds': None,
                                         'last_sim_time_seconds': None})
        now = self.get_clock().now().nanoseconds / 1e9
        event['count'] += 1
        event['first_sim_time_seconds'] = event['first_sim_time_seconds'] or now
        event['last_sim_time_seconds'] = now

    def _feedback_cb(self, message):
        if self.case_record is None or self.stage != 'goal':
            return
        feedback = message.feedback
        self.case_record['nav2_feedback'] = {
            'distance_remaining': float(feedback.distance_remaining),
            'recoveries': int(feedback.number_of_recoveries),
            'current_pose': self._pose_dict(feedback.current_pose.pose.position,
                                            feedback.current_pose.pose.orientation)}

    def _snapshot(self):
        snapshot = {'sim_time_seconds': self.get_clock().now().nanoseconds / 1e9,
                    'map': None, 'world': None, 'odom': None}
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            snapshot['map'] = self._pose_dict(transform.transform.translation,
                                              transform.transform.rotation)
        except Exception:
            pass
        if self.world_pose_received_at is not None and time.monotonic() - self.world_pose_received_at < 3.0:
            snapshot['world'] = dict(self.world_pose)
        if snapshot['map'] is not None:
            snapshot['world_estimate'] = self._world_estimate_from_map(
                (snapshot['map']['x'], snapshot['map']['y']))
        if self.odom is not None:
            snapshot['odom'] = self._pose_dict(self.odom.pose.pose.position,
                                                self.odom.pose.pose.orientation)
        return snapshot

    def _map_info(self):
        if self.map_msg is None:
            return None
        grid = self.map_msg
        return {'frame_id': grid.header.frame_id,
                'stamp': {'sec': grid.header.stamp.sec, 'nanosec': grid.header.stamp.nanosec},
                'sha256': hashlib.sha256(bytes(value + 1 for value in grid.data)).hexdigest(),
                'data': list(grid.data), 'resolution': grid.info.resolution,
                'width': grid.info.width, 'height': grid.info.height,
                'origin': self._pose_dict(grid.info.origin.position,
                                          grid.info.origin.orientation)}

    @staticmethod
    def _world_goal_estimate(goal_map, snapshot):
        map_pose, world_pose = snapshot['map'], snapshot['world']
        if map_pose is None or world_pose is None:
            return None
        dx, dy = goal_map[0] - map_pose['x'], goal_map[1] - map_pose['y']
        rotation = world_pose['yaw'] - map_pose['yaw']
        return {'x': world_pose['x'] + math.cos(rotation) * dx - math.sin(rotation) * dy,
                'y': world_pose['y'] + math.sin(rotation) * dx + math.cos(rotation) * dy,
                'yaw': rotation,
                'method': 'instantaneous_map_world_alignment_estimate'}

    @staticmethod
    def _robot_spawn_pose():
        path = os.path.join(get_package_share_directory('robot_description'),
                            'worlds', 'casa.sdf')
        world = ET.parse(path).getroot()
        robot = next((item for item in world.findall('.//include')
                      if item.findtext('name') == 'aspirador'), None)
        if robot is None:
            raise ValueError('aspirador include missing from casa.sdf')
        pose = [float(value) for value in robot.findtext('pose', '0 0 0 0 0 0').split()]
        return {'x': pose[0], 'y': pose[1], 'z': pose[2], 'yaw': pose[5],
                'source': path}

    def _world_estimate_from_map(self, map_xy):
        spawn = self.world_spawn_pose
        c, s = math.cos(spawn['yaw']), math.sin(spawn['yaw'])
        return {'x': spawn['x'] + c * map_xy[0] - s * map_xy[1],
                'y': spawn['y'] + s * map_xy[0] + c * map_xy[1],
                'yaw': spawn['yaw'],
                'method': 'SDF_spawn_pose_plus_map_offset',
                'source': spawn['source']}

    def _sample_route(self):
        if self.case_record is None or self.stage != 'goal' or self.future is None:
            return
        now = time.monotonic()
        if now - self.last_route_sample >= 0.5:
            self.case_record['executed_route'].append(self._snapshot())
            self.last_route_sample = now

    def _map_cell(self, x, y):
        grid = self.map_msg
        if grid is None or not grid.data:
            return 'no_map'
        origin = grid.info.origin.position
        q = grid.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        dx, dy = x-origin.x, y-origin.y
        gx = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/grid.info.resolution)
        gy = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/grid.info.resolution)
        if not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height):
            return 'out_of_bounds'
        return str(grid.data[gy*grid.info.width+gx])

    def _tick(self):
        if self.finished:
            return
        if self.robot_state != 'IDLE':
            now = time.monotonic()
            if now-self.last_pause_request >= 1.0:
                self.control_pub.publish(String(data='PAUSE'))
                self.last_pause_request = now
            if now-self.started_wall > self.startup_timeout:
                if self.control_index < 3:
                    for name in ('forward', 'reverse', 'in_place_turn'):
                        self.results.append({'scenario': name, 'passed': False,
                                             'observed': 'autonomy_pause_timeout'})
                for case in self.cases:
                    self.results.append({'scenario': case['name'], 'passed': False,
                                         'observed': 'autonomy_pause_timeout'})
                self._write_report()
                self.finished = True
            return
        if time.monotonic()-self.started_wall > self.startup_timeout and (self.odom is None or self.control_index < 3):
            for name in ('forward', 'reverse', 'in_place_turn'):
                if not any(row['scenario'] == name for row in self.results):
                    self.results.append({'scenario': name, 'passed': False, 'observed': 'odometry_timeout'})
            for case in self.cases:
                self.results.append({'scenario': case['name'], 'passed': False, 'observed': 'startup_timeout'})
            self._write_report()
            self.finished = True
            return
        self._sample_route()
        if self.pending_outcome is not None:
            self._settle_stage()
            return
        # Wait for Nav2's final zero command before starting a mobility probe.
        # Once a probe is active, its own nonzero cmd_vel must not block the
        # next tick; otherwise the runner deadlocks immediately after publishing
        # the first control command.
        control_active = self.control_index < 3 and self.control_start is not None
        navigation_active = self.future is not None
        if not control_active and not navigation_active:
            if (abs(self.cmd.linear.x) > 0.02 or abs(self.cmd.linear.y) > 0.02 or
                    abs(self.cmd.angular.z) > 0.04):
                self.idle_clear_since = None
                return
            if self.idle_clear_since is None:
                self.idle_clear_since = time.monotonic()
                return
            if time.monotonic()-self.idle_clear_since < 0.5:
                return
        if self._run_control_test():
            return
        if self.index >= len(self.cases):
            self._write_report()
            self.finished = True
            return
        if not self.map_seen or not self.client.server_is_ready():
            if not self.client.server_is_ready():
                self.client.wait_for_server(timeout_sec=0.05)
            if time.monotonic()-self.started_wall > self.startup_timeout:
                for case in self.cases:
                    self.results.append({'scenario': case['name'], 'passed': False, 'observed': 'map_or_nav2_timeout'})
                self._write_report()
                self.finished = True
            return
        if self.case_record is None:
            current = self._snapshot()['map']
            if current is not None and self.index < len(self.cases) - 1:
                closest = min(range(self.index, len(self.cases)), key=lambda idx:
                              math.hypot(self.cases[idx]['start_map'][0] - current['x'],
                                         self.cases[idx]['start_map'][1] - current['y']))
                self.cases[self.index], self.cases[closest] = (
                    self.cases[closest], self.cases[self.index])
            case = self.cases[self.index]
            self.case_record = {
                'scenario': case['name'], 'goal_map': case['goal_map'],
                'start_target_map': case['start_map'], 'feature_world': case['feature_world'],
                'goal_cell': self._map_cell(*case['goal_map']), 'map_info': self._map_info(),
                'setup_initial': self._snapshot(),
                'start': None, 'end': None, 'executed_route': [],
                'planned_route_map': [], 'mechanical_failure_observed': False,
                'mechanical_evidence': 'no_contact_sensor_or_ground_truth_force',
                'collision_monitor_events': {}, 'plan_update_count': 0,
                'goal_timeout_seconds': self.timeout,
                'setup_timeout_seconds': self.setup_timeout}
            self.setup_targets = list(case.get('setup_waypoints_map', [])) + [case['start_map']]
            self.setup_target_index = 0
            self.case_record['setup_steps'] = []
            self.stage = 'setup'
        else:
            case = self.cases[self.index]
        if self.future is None:
            if self.stage == 'setup':
                pose = self._snapshot()['map']
                x, y = self.setup_targets[self.setup_target_index]
                target_yaw = self._setup_yaw(case)
                at_target = (pose is not None and
                             math.hypot(pose['x'] - x, pose['y'] - y) <= 0.20 and
                             abs(self._yaw_error(pose['yaw'], target_yaw)) <= 0.35)
                if at_target:
                    self.case_record['setup_steps'].append({
                        'target_map': [x, y], 'outcome': 'already_at_target',
                        'end': self._snapshot(), 'target_yaw_rad': target_yaw,
                        'yaw_error_to_target_rad': self._yaw_error(pose['yaw'], target_yaw),
                        'accepted_by_pose_tolerance': True})
                    self.setup_target_index += 1
                    if self.setup_target_index >= len(self.setup_targets):
                        self.case_record['setup'] = {'outcome': 'pose_tolerance_reached',
                                                     'nav2_outcome': 'not_required',
                                                     'end': self._snapshot()}
                        self._begin_trial(case)
                    return
            self._send_stage_goal(case)
            return
        if self.future.done():
            try:
                if not self.waiting_for_result:
                    handle = self.future.result()
                    if not handle.accepted:
                        self._start_settle('goal_rejected', None, None)
                        return
                    self.goal_handle = handle
                    self.future = handle.get_result_async()
                    self.waiting_for_result = True
                    return
                wrapped = self.future.result()
                status = int(wrapped.status)
                error_code = getattr(wrapped.result, 'error_code', None)
                outcome = 'goal_reached' if status == GoalStatus.STATUS_SUCCEEDED else 'nav2_failure'
                if outcome != 'goal_reached' and self.stage == 'goal':
                    cell = self.case_record['goal_cell']
                    if cell == 'out_of_bounds' or (cell.lstrip('-').isdigit() and int(cell) >= 65):
                        outcome = 'invalid_goal'
                self._start_settle(outcome, status, error_code,
                                   getattr(wrapped.result, 'error_msg', None))
                return
            except Exception as exc:
                self._start_settle('exception', None, None, str(exc))
                return
        limit = self.setup_timeout if self.stage == 'setup' else self.timeout
        if self.goal_sent_at and time.monotonic()-self.goal_sent_at > limit:
            self._start_settle('timeout', None, None)
            return

    def _run_control_test(self):
        controls = [
            ('reverse', -0.12, 0.0, 'reverse'),
            ('in_place_turn', 0.0, 0.8, 'turn'),
            # Back away and reorient before the forward probe so a robot left
            # against furniture or a wall after exploration can clear it first.
            ('forward', 0.12, 0.0, 'forward'),
        ]
        if self.control_index >= len(controls):
            return False
        if self.odom is None:
            return True
        name, linear, angular, kind = controls[self.control_index]
        now = self.get_clock().now()
        if self.control_start is None:
            self.control_start = now
            self.control_pose = self.odom.pose.pose
        elapsed = (now-self.control_start).nanoseconds/1e9
        command = Twist()
        if elapsed < 2.5:
            command.linear.x = linear
            command.angular.z = angular
            self.cmd_pub.publish(command)
            return True
        self.cmd_pub.publish(Twist())
        current = self.odom.pose.pose
        dx = current.position.x-self.control_pose.position.x
        dy = current.position.y-self.control_pose.position.y
        old = self.control_pose.orientation
        new = current.orientation
        yaw0 = math.atan2(2*(old.w*old.z+old.x*old.y), 1-2*(old.y*old.y+old.z*old.z))
        yaw1 = math.atan2(2*(new.w*new.z+new.x*new.y), 1-2*(new.y*new.y+new.z*new.z))
        dyaw = math.atan2(math.sin(yaw1-yaw0), math.cos(yaw1-yaw0))
        forward_distance = dx*math.cos(yaw0) + dy*math.sin(yaw0)
        if kind == 'forward':
            passed, observed = forward_distance > 0.08, 'forward=%.3f m' % forward_distance
        elif kind == 'reverse':
            passed, observed = forward_distance < -0.08, 'forward=%.3f m' % forward_distance
        else:
            passed, observed = abs(dyaw) > 0.8, 'dyaw=%.3f rad' % dyaw
        self.results.append({'scenario': name, 'passed': bool(passed), 'observed': observed})
        self.get_logger().info('%s: %s (%s)' % (name, 'PASS' if passed else 'FAIL', observed))
        self.control_index += 1
        self.control_start = None
        self.control_pose = None
        return True

    def _send_stage_goal(self, case):
        x, y = self.setup_targets[self.setup_target_index] if self.stage == 'setup' else case['goal_map']
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = self._setup_yaw(case) if self.stage == 'setup' else 0.0
        goal.pose.pose.orientation = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        self.future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        self.goal_sent_at = time.monotonic()
        self.waiting_for_result = False
        self.goal_handle = None
        self.get_logger().info('%s %s: (%.2f, %.2f), map_cell=%s' %
                               (case['name'], self.stage, x, y, self._map_cell(x, y)))

    def _begin_trial(self, case):
        self.stage = 'goal'
        start = self._snapshot()
        self.case_record['start'] = start
        self.case_record['goal_world_estimate'] = (
            self._world_goal_estimate(case['goal_map'], start) or
            self._world_estimate_from_map(case['goal_map']))
        self.case_record['goal_cell_at_start'] = self._map_cell(*case['goal_map'])
        map_pose = start['map']
        if map_pose is not None:
            distance = math.hypot(case['goal_map'][0] - map_pose['x'],
                                  case['goal_map'][1] - map_pose['y'])
            self.case_record['straight_line_m'] = distance
            self.case_record['minimum_drive_seconds_at_0_14_mps'] = distance / 0.14
        self.case_record['map_info_at_start'] = self._map_info()
        self.case_record['executed_route'] = [start]
        self.case_record['planned_route_map'] = []
        self.last_route_sample = time.monotonic()

    def _start_settle(self, outcome, status, error_code, detail=None):
        self.pending_outcome = {'outcome': outcome, 'nav2_status': status,
                                'nav2_error_code': error_code, 'detail': detail,
                                'elapsed_wall_seconds': None if self.goal_sent_at is None else
                                time.monotonic() - self.goal_sent_at}
        self.settle_started = time.monotonic()
        self.stopped_since = None
        self.cancel_future = None

    def _abort_unsettled(self):
        row = self.case_record
        row.update({'passed': False, 'outcome': 'cancel_not_confirmed',
                    'observed': 'Nav2 goal could still be active; suite stopped',
                    'end': self._snapshot()})
        self.results.append(row)
        for case in self.cases[self.index + 1:]:
            self.results.append({'scenario': case['name'], 'passed': False,
                                 'outcome': 'not_run_unsafe_cancellation'})
        self._write_report()
        self.finished = True

    def _settle_stage(self):
        if self.pending_outcome['outcome'] == 'timeout':
            if self.goal_handle is None and self.future is not None:
                if not self.future.done():
                    if time.monotonic() - self.settle_started > 8.0:
                        self._abort_unsettled()
                    return
                try:
                    handle = self.future.result()
                except Exception:
                    handle = None
                if handle is not None and handle.accepted:
                    self.goal_handle = handle
                    self.future = handle.get_result_async()
                    self.waiting_for_result = True
            if self.goal_handle is not None:
                if self.future is None or not self.future.done():
                    if self.cancel_future is None:
                        self.cancel_future = self.goal_handle.cancel_goal_async()
                    if self.cancel_future.done() and not self.cancel_future.result().goals_canceling:
                        self._abort_unsettled()
                        return
                    if time.monotonic() - self.settle_started > 8.0:
                        self._abort_unsettled()
                    return
        moving = (abs(self.cmd.linear.x) > 0.02 or abs(self.cmd.angular.z) > 0.04)
        if self.odom is not None:
            moving = moving or abs(self.odom.twist.twist.linear.x) > 0.02
        if moving:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = time.monotonic()
        if self.stopped_since is None or time.monotonic() - self.stopped_since < 0.5:
            if time.monotonic() - self.settle_started > 8.0:
                self._abort_unsettled()
            return
        self._complete_stage()

    def _threshold_crossing(self, case):
        feature = case['feature_world']
        worlds = [point['world'] for point in self.case_record['executed_route']
                  if point['world'] is not None]
        if len(worlds) < 2:
            return {'observed': False, 'reason': 'world_pose_unavailable'}
        cx, cy = feature['pose']
        axis = feature.get('crossing_axis', 'y')
        if axis == 'x':
            half_depth, half_width = feature['size'][0] / 2, feature['size'][1] / 2
            approach = any(p['x'] < cx - half_depth for p in worlds)
            departure = any(p['x'] > cx + half_depth for p in worlds)
            centered = any(abs(p['x'] - cx) <= 0.10 and abs(p['y'] - cy) <= half_width
                           for p in worlds)
            climb = max((p['z'] - worlds[0]['z'] for p in worlds
                         if abs(p['x'] - cx) <= half_depth + 0.015 and
                         abs(p['y'] - cy) <= half_width), default=0.0)
        else:
            half_depth = feature['size'][1] / 2
            approach = any(p['y'] < cy - half_depth for p in worlds)
            departure = any(p['y'] > cy + half_depth for p in worlds)
            centered = any(abs(p['x'] - cx) <= 0.10 and abs(p['y'] - cy) <= half_depth
                           for p in worlds)
            climb = max((p['z'] - worlds[0]['z'] for p in worlds
                         if abs(p['x'] - cx) <= 0.15 and abs(p['y'] - cy) <= half_depth),
                        default=0.0)
        return {'observed': approach and departure and centered and climb >= 0.005,
                'axis': axis, 'approach': approach, 'departure': departure,
                'centered': centered, 'height_gain_m': climb}

    def _diagnose_failure(self, case, outcome, end):
        """Attach evidence-based causes; distinguish navigation evidence from contact."""
        result = outcome['outcome']
        cell = self.case_record.get('goal_cell_at_start', self.case_record.get('goal_cell'))
        planned = len(self.case_record.get('planned_route_map', []))
        events = self.case_record.get('collision_monitor_events', {})
        triggered_actions = {item['action_type'] for item in events.values()}
        start, finish = self.case_record.get('start'), end
        map_pose = finish.get('map') if finish else None
        world_pose = finish.get('world') if finish else None
        localization_error = None
        if map_pose is not None and world_pose is not None:
            expected = self._world_estimate_from_map((map_pose['x'], map_pose['y']))
            localization_error = math.hypot(expected['x'] - world_pose['x'],
                                             expected['y'] - world_pose['y'])
        if result == 'invalid_goal':
            category = 'invalid_goal'
            evidence = {'goal_cell_at_start': cell}
        elif result in ('timeout', 'nav2_failure'):
            if localization_error is not None and localization_error > 0.35:
                category = 'localization_inconsistency'
            elif planned < 2:
                category = 'planning_or_costmap_no_route'
            elif triggered_actions:
                category = 'obstacle_interference_proxy'
            elif (self.case_record.get('nav2_feedback') or {}).get('recoveries', 0) > 0:
                category = 'controller_recovery_exhausted'
            elif result == 'timeout':
                category = 'timeout_with_plan_and_no_recovery_evidence'
            else:
                category = 'nav2_failure_after_plan'
            evidence = {
                'planned_route_points': planned,
                'plan_update_count': self.case_record.get('plan_update_count', 0),
                'collision_monitor_events': events,
                'nav2_feedback': self.case_record.get('nav2_feedback'),
                'map_world_position_error_m': localization_error,
                'goal_cell_at_start': cell,
                'mechanical_contact_confirmed': False,
            }
        else:
            category = 'not_applicable'
            evidence = {'outcome': result}
        return {'category': category, 'evidence': evidence,
                'confidence': 'medium' if category not in
                ('obstacle_interference_proxy', 'timeout_with_plan_and_no_recovery_evidence')
                else 'low'}

    def _complete_stage(self):
        case = self.cases[self.index]
        outcome = self.pending_outcome
        end = self._snapshot()
        self.future = None
        self.goal_handle = None
        self.goal_sent_at = None
        self.pending_outcome = None
        self.cancel_future = None
        self.idle_clear_since = None
        if self.stage == 'setup':
            pose = end['map']
            target = self.setup_targets[self.setup_target_index]
            distance = None if pose is None else math.hypot(pose['x'] - target[0], pose['y'] - target[1])
            target_yaw = self._setup_yaw(case)
            yaw_error = None if pose is None else self._yaw_error(pose['yaw'], target_yaw)
            setup_ok = (outcome['outcome'] == 'goal_reached' and distance is not None and
                        distance <= 0.25 and yaw_error is not None and abs(yaw_error) <= 0.35)
            prior_misses = sum(
                step.get('target_map') == target and step.get('outcome') == 'goal_reached'
                and not step.get('accepted_by_pose_tolerance', False)
                for step in self.case_record['setup_steps'])
            step = {'target_map': target, **outcome, 'end': end,
                    'distance_to_target_m': distance,
                    'target_yaw_rad': target_yaw,
                    'yaw_error_to_target_rad': yaw_error,
                    'attempt': prior_misses + 1,
                    'accepted_by_pose_tolerance': setup_ok}
            self.case_record['setup_steps'].append(step)
            if setup_ok:
                self.setup_target_index += 1
                if self.setup_target_index < len(self.setup_targets):
                    return
                self.case_record['setup'] = {'outcome': 'target_tolerance_reached',
                                             'nav2_outcome': outcome['outcome'], 'end': end,
                                             'distance_to_start_m': distance}
                self._begin_trial(case)
                return
            if (outcome['outcome'] == 'goal_reached' and
                    prior_misses < self.setup_retry_limit):
                self.get_logger().warning(
                    '%s setup pose outside tolerance (%.3f m, yaw %.3f rad); retry %d/%d' %
                    (case['name'], distance if distance is not None else float('inf'),
                     yaw_error if yaw_error is not None else float('inf'),
                     prior_misses + 1, self.setup_retry_limit))
                return
            self.case_record['setup'] = {**outcome, 'end': end,
                                         'distance_to_target_m': distance,
                                         'yaw_error_to_target_rad': yaw_error,
                                         'attempts': prior_misses + 1,
                                         'retry_limit': self.setup_retry_limit}
            self.case_record.update({'passed': False, 'outcome': 'setup_failed',
                                     'observed': outcome['outcome'], 'end': end})
        else:
            self.case_record['executed_route'].append(end)
            self.case_record['map_info_at_end'] = self._map_info()
            classified = outcome['outcome']
            crossing = None
            if classified == 'goal_reached' and case.get('requires_frontal_crossing'):
                crossing = self._threshold_crossing(case)
                if not crossing['observed']:
                    classified = 'route_not_proven'
            passed = _matches_expected(case['expect'], classified)
            self.case_record.update({'passed': passed, 'outcome': classified,
                                     'observed': outcome['outcome'], 'end': end,
                                     'elapsed_wall_seconds': outcome['elapsed_wall_seconds'],
                                     'nav2_status': outcome['nav2_status'],
                                     'nav2_error_code': outcome['nav2_error_code'],
                                     'detail': outcome['detail']})
            if not passed:
                self.case_record['failure_diagnosis'] = self._diagnose_failure(case, outcome, end)
            if crossing is not None:
                self.case_record['threshold_crossing'] = crossing
        row = self.case_record
        self.results.append(row)
        self.get_logger().info('%s: %s (%s)' %
                               (case['name'], 'PASS' if row['passed'] else 'FAIL', row['outcome']))
        self.index += 1
        self.case_record = None
        self.stage = 'setup'
        self.setup_targets = []
        self.setup_target_index = 0

    def _write_report(self):
        os.makedirs(self.output_dir, exist_ok=True)
        report = {'suite': 'mvp1_mobility', 'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  'passed': sum(1 for x in self.results if x['passed']), 'failed': sum(1 for x in self.results if not x['passed']),
                  'scenarios': self.results,
                  'timeouts_seconds': {'goal': self.timeout, 'setup': self.setup_timeout,
                                       'startup': self.startup_timeout},
                  'notes': 'Each case first navigates Nav2 to its declared start pose. Remaining scenarios are ordered by distance from the last observed start pose to reduce long setup transits. Setup failures are separate from invalid goals, Nav2 failures, and timeouts. world_estimate uses the initial spawn transform from the SDF; world is populated only when Gazebo supplies a usable world pose. A blocked goal exercises planning/recovery, not physical entrapment. Without contact sensing, mechanical failure cannot be confirmed.',
                  'world_pose_source': '/model/aspirador/pose (Gazebo Pose_V bridged to TFMessage)',
                  'world_spawn_sdf': [self.world_spawn_pose['x'], self.world_spawn_pose['y'],
                                      self.world_spawn_pose['z']],
                  'world_spawn_sdf_source': self.world_spawn_pose['source']}
        path = os.path.join(self.output_dir, 'mobility_report.json')
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
        self.get_logger().info('Relatório salvo: ' + path)


def main():
    rclpy.init()
    node = MobilityRunner()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    failures = sum(1 for row in node.results if not row['passed'])
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()
