import json
import math
import os
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class BatteryCycleRunner(Node):
    def __init__(self):
        super().__init__('battery_cycle_runner')
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('timeout_seconds', 300.0)
        self.declare_parameter('resume_battery_percent', 97.0)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.timeout = float(self.get_parameter('timeout_seconds').value)
        self.resume_threshold = float(self.get_parameter('resume_battery_percent').value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot/state', self._state_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(BatteryState, '/battery_state', self._battery_cb, 10)
        self.create_subscription(String, '/robot/event', self._event_cb, 20)
        self.states = []
        self.dock_start_xy = None
        self.return_xy = None
        self.odom_xy = None
        self.return_distance = None
        self.battery_percent = None
        self.battery_at_return = None
        self.battery_at_charge_start = None
        self.battery_at_charge_end = None
        self.battery_at_resume = None
        self.battery_reached_resume_threshold = False
        self.charge_peak_percent = None
        self.charge_samples = 0
        self.dock_assessment = None
        self.failure_event = None
        self.started = time.monotonic()
        self.finished = False
        self.passed = False
        self.failure = None
        self.create_timer(0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _state_cb(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)
            self.get_logger().info('Estado observado: ' + msg.data)
            if msg.data == 'RETURNING_TO_DOCK':
                self.battery_at_return = self.battery_percent
                self.return_xy = self.odom_xy
                if self.dock_start_xy is not None and self.return_xy is not None:
                    self.return_distance = ((self.return_xy[0]-self.dock_start_xy[0])**2 +
                                            (self.return_xy[1]-self.dock_start_xy[1])**2)**0.5
            elif msg.data == 'CHARGING':
                self.battery_at_charge_start = self.battery_percent
            elif msg.data in ('EXPLORING', 'COVERING') and 'CHARGING' in self.states[:-1]:
                self.battery_at_resume = self.battery_percent
                self.battery_at_charge_end = self.battery_percent

    def _battery_cb(self, msg):
        if math.isfinite(msg.percentage) and 0.0 <= msg.percentage <= 1.0:
            self.battery_percent = msg.percentage * 100.0
            if self.states and self.states[-1] == 'CHARGING':
                if self.battery_at_charge_start is None:
                    self.battery_at_charge_start = self.battery_percent
                self.charge_peak_percent = max(self.charge_peak_percent or 0.0, self.battery_percent)
                self.charge_samples += 1
                if self.battery_percent >= self.resume_threshold:
                    self.battery_reached_resume_threshold = True

    def _event_cb(self, msg):
        if msg.data.startswith('dock_assessment:'):
            try:
                self.dock_assessment = json.loads(msg.data.split(':', 1)[1])
            except (ValueError, TypeError):
                self.failure_event = 'invalid_dock_assessment'
        elif msg.data in ('dock_navigation_failed', 'dock_pose_unavailable'):
            self.failure_event = msg.data

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        if self.dock_start_xy is None:
            self.dock_start_xy = self.odom_xy

    def _tick(self):
        if self.finished:
            return
        if 'ERROR' in self.states:
            self.failure = self.failure_event or self._phase_failure('autonomy_entered_error')
            self._finish()
        elif ('RETURNING_TO_DOCK' in self.states and 'CHARGING' in self.states and
              any(state in ('EXPLORING', 'COVERING')
                  for state in self.states[self.states.index('CHARGING')+1:])):
            self.failure = self._completion_failure()
            self.passed = self.failure is None
            self._finish()
        elif time.monotonic()-self.started >= self.timeout:
            if 'RETURNING_TO_DOCK' not in self.states:
                self.failure = 'timeout_waiting_for_return'
            elif 'CHARGING' not in self.states:
                self.failure = self.failure_event or self._phase_failure('timeout')
            else:
                self.failure = ('timeout_resume' if self.battery_reached_resume_threshold
                                else 'timeout_charging')
            self._finish()

    def _phase_failure(self, reason):
        if 'CHARGING' in self.states:
            return 'charging_%s' % reason
        if 'RETURNING_TO_DOCK' not in self.states:
            return 'navigation_%s' % reason
        if 'DOCKING' in self.states:
            return 'docking_%s' % reason
        return 'navigation_%s' % reason

    def _completion_failure(self):
        if self.failure_event:
            return self.failure_event
        if self.return_distance is None:
            return 'return_distance_unavailable'
        if 'DOCKING' not in self.states:
            return 'docking_state_not_observed'
        dock = self.dock_assessment
        if not isinstance(dock, dict) or not all(k in dock for k in
                ('distance_m', 'speed_mps', 'radius_limit_m', 'speed_limit_mps', 'reference', 'frame_id')):
            return 'dock_measurement_unavailable'
        if not all(isinstance(dock[k], (int, float)) and not isinstance(dock[k], bool) and
                   math.isfinite(dock[k]) for k in
                ('distance_m', 'speed_mps', 'radius_limit_m', 'speed_limit_mps')):
            return 'dock_measurement_invalid'
        if (dock['distance_m'] < 0 or dock['speed_mps'] < 0 or
                dock['radius_limit_m'] <= 0 or dock['speed_limit_mps'] <= 0 or
                not isinstance(dock['reference'], str) or not dock['reference'].strip() or
                not isinstance(dock['frame_id'], str) or not dock['frame_id'].strip()):
            return 'dock_measurement_invalid'
        if dock['distance_m'] > dock['radius_limit_m'] or dock['speed_mps'] > dock['speed_limit_mps']:
            return 'dock_limits_exceeded'
        if any(value is None for value in (self.battery_at_return, self.battery_at_charge_start,
                                            self.battery_at_charge_end, self.battery_at_resume,
                                            self.charge_peak_percent)):
            return 'battery_measurement_unavailable'
        if self.charge_samples < 2 or self.charge_peak_percent <= self.battery_at_charge_start + 0.05:
            return 'charge_did_not_increase'
        if self.battery_at_charge_end < self.resume_threshold:
            return 'charge_ended_below_battery_threshold'
        if self.battery_at_resume < self.resume_threshold:
            return 'resumed_below_battery_threshold'
        return None

    def _finish(self):
        os.makedirs(self.output_dir, exist_ok=True)
        report = {'suite': 'mvp1_battery_docking', 'passed': self.passed,
                  'elapsed_seconds': round(time.monotonic()-self.started, 2),
                  'states': self.states, 'return_distance_m': self.return_distance,
                  'dock_assessment': self.dock_assessment,
                  'dock_reference_note': 'Virtual zone at the initial robot pose; no alignment or physical contact with base_charging is measured. The SDF dock center is 0.63 m from the robot spawn.',
                  'battery_percent': {'return_start': self.battery_at_return,
                                      'charge_start': self.battery_at_charge_start,
                                      'charge_peak': self.charge_peak_percent,
                                      'charge_end': self.battery_at_charge_end,
                                      'resume': self.battery_at_resume,
                                      'resume_threshold': self.resume_threshold},
                  'charge_samples': self.charge_samples,
                  'failure': self.failure}
        path = os.path.join(self.output_dir, 'battery_report.json')
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
        self.get_logger().info('Relatório salvo: ' + path)
        self.finished = True


def main():
    rclpy.init()
    node = BatteryCycleRunner()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    passed = node.passed
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
