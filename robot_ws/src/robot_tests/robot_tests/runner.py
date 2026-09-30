import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MobilityRunner(Node):
    def __init__(self):
        super().__init__('mobility_runner')
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('goal_timeout', 100.0)
        self.declare_parameter('startup_timeout', 120.0)
        self.declare_parameter('scenario_limit', 8)
        self.declare_parameter('scenario_name', '')
        self.declare_parameter('run_control_tests', True)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.timeout = float(self.get_parameter('goal_timeout').value)
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        scenarios_path = os.path.join(get_package_share_directory('robot_tests'), 'scenarios', 'mobility.json')
        with open(scenarios_path, encoding='utf-8') as stream:
            self.cases = json.load(stream)
        scenario_name = str(self.get_parameter('scenario_name').value)
        if scenario_name:
            self.cases = [case for case in self.cases if case['name'] == scenario_name]
        else:
            self.cases = self.cases[:max(1, int(self.get_parameter('scenario_limit').value))]
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_pub = self.create_publisher(String, '/robot/control', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(self, 'odom', m), 10)
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
        self.results = []
        self.finished = False
        self.started_wall = time.monotonic()
        self.control_index = 0
        if not bool(self.get_parameter('run_control_tests').value):
            self.control_index = 3
        self.control_start = None
        self.control_pose = None
        self.create_timer(0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _map_cb(self, msg):
        self.map_seen = True
        self.map_msg = msg

    def _map_cell(self, x, y):
        grid = self.map_msg
        if grid is None or not grid.data:
            return 'no_map'
        origin = grid.info.origin.position
        q = grid.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        dx, dy = x-origin.x, y-origin.y
        gx = int((math.cos(yaw)*dx+math.sin(yaw)*dy)/grid.info.resolution)
        gy = int((-math.sin(yaw)*dx+math.cos(yaw)*dy)/grid.info.resolution)
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
        case = self.cases[self.index]
        name, (x, y), expect_success = case['name'], case['goal_map'], case['expect'] == 'success'
        if self.future is None:
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation = Quaternion(w=1.0)
            self.future = self.client.send_goal_async(goal)
            self.waiting_for_result = False
            self.goal_sent_at = time.monotonic()
            self.get_logger().info('Cenário %s: objetivo (%.2f, %.2f), célula do mapa=%s' %
                                   (name, x, y, self._map_cell(x, y)))
            return
        if self.future.done():
            try:
                if not self.waiting_for_result:
                    handle = self.future.result()
                    if not handle.accepted:
                        self._finish_case(name, False, 'rejected')
                        return
                    self.goal_handle = handle
                    self.future = handle.get_result_async()
                    self.waiting_for_result = True
                    return
                wrapped = self.future.result()
                success = wrapped.status == GoalStatus.STATUS_SUCCEEDED
                observed = 'success' if success else 'failure; goal_cell=' + self._map_cell(*case['goal_map'])
                self._finish_case(name, success == expect_success, observed)
                return
            except Exception as exc:
                self._finish_case(name, False, 'exception: ' + str(exc))
                return
        if self.goal_sent_at and time.monotonic()-self.goal_sent_at > self.timeout:
            # An intentionally unreachable goal passes when Nav2 either
            # rejects it, aborts it, or reaches this bounded timeout.
            expected_failure = not expect_success
            self._finish_case(name, expected_failure,
                              'expected_failure_timeout' if expected_failure else 'timeout')
            return

    def _run_control_test(self):
        controls = [
            ('forward', 0.12, 0.0, 'forward'),
            ('reverse', -0.12, 0.0, 'reverse'),
            ('in_place_turn', 0.0, 0.8, 'turn'),
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

    def _finish_case(self, name, passed, observed):
        if observed in ('timeout', 'expected_failure_timeout') and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.results.append({'scenario': name, 'passed': bool(passed), 'observed': observed})
        self.get_logger().info('%s: %s (%s)' % (name, 'PASS' if passed else 'FAIL', observed))
        self.index += 1
        self.future = None
        self.goal_handle = None
        self.goal_sent_at = None

    def _write_report(self):
        os.makedirs(self.output_dir, exist_ok=True)
        report = {'suite': 'mvp1_mobility', 'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  'passed': sum(1 for x in self.results if x['passed']), 'failed': sum(1 for x in self.results if not x['passed']),
                  'scenarios': self.results,
                  'notes': 'Scenarios use Nav2 goals in the shared casa.sdf world. A blocked-goal failure exercises planner/recovery handling, not physical entrapment.'}
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
