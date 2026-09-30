import json
import os
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class BatteryCycleRunner(Node):
    def __init__(self):
        super().__init__('battery_cycle_runner')
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('timeout_seconds', 300.0)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.timeout = float(self.get_parameter('timeout_seconds').value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot/state', self._state_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.states = []
        self.dock_start_xy = None
        self.return_xy = None
        self.odom_xy = None
        self.return_distance = None
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
                self.return_xy = self.odom_xy
                if self.dock_start_xy is not None and self.return_xy is not None:
                    self.return_distance = ((self.return_xy[0]-self.dock_start_xy[0])**2 +
                                            (self.return_xy[1]-self.dock_start_xy[1])**2)**0.5

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        if self.dock_start_xy is None:
            self.dock_start_xy = self.odom_xy

    def _tick(self):
        if self.finished:
            return
        if 'ERROR' in self.states:
            self.failure = 'autonomy_entered_error'
            self._finish()
        elif ('RETURNING_TO_DOCK' in self.states and 'CHARGING' in self.states and
              'EXPLORING' in self.states[self.states.index('CHARGING')+1:]):
            self.passed = self.return_distance is not None and self.return_distance >= 0.8
            if not self.passed:
                self.failure = 'robot_did_not_leave_dock_by_0.8m'
            self._finish()
        elif time.monotonic()-self.started >= self.timeout:
            self.failure = 'timeout_waiting_for_return_charge_and_resume'
            self._finish()

    def _finish(self):
        os.makedirs(self.output_dir, exist_ok=True)
        report = {'suite': 'mvp1_battery_docking', 'passed': self.passed,
                  'elapsed_seconds': round(time.monotonic()-self.started, 2),
                  'states': self.states, 'return_distance_m': self.return_distance,
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
