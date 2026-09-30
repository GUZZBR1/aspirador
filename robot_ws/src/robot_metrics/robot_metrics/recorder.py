import json
import math
import os
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        self.file = open(self.path, 'a', encoding='utf-8', buffering=1)
        self.start_time = self.get_clock().now()
        self.odom = None
        self.battery = None
        self.coverage = 0.0
        self.state = 'UNKNOWN'
        self.cmd = Twist()
        self.distance = 0.0
        self.last_xy = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.events = {'collision_proxy': 0, 'recovery_attempts': 0, 'stuck_locations': [], 'return_seconds': []}
        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 20)
        self.create_subscription(BatteryState, '/battery_state', lambda m: setattr(self, 'battery', m.percentage), 10)
        self.create_subscription(Float32, '/coverage/percent', lambda m: setattr(self, 'coverage', m.data), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot/state', lambda m: setattr(self, 'state', m.data), latched)
        self.create_subscription(String, '/robot/event', self._event, 20)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_timer(1.0, self._write_sample)
        self.get_logger().info('Gravando telemetria em ' + self.path)

    def _odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_xy:
            self.distance += math.hypot(xy[0]-self.last_xy[0], xy[1]-self.last_xy[1])
        self.last_xy = xy

    def _event(self, msg):
        text = msg.data
        if text.startswith('collision_proxy'):
            self.events['collision_proxy'] += 1
        elif text.startswith('recovery_attempt'):
            self.events['recovery_attempts'] += 1
        elif text == 'stuck_detected' and self.odom:
            pose, frame = self._position_pose()
            self.events['stuck_locations'].append({'x': pose.position.x, 'y': pose.position.y,
                                                   'frame_id': frame})
        elif text.startswith('return_to_dock_seconds:'):
            try:
                self.events['return_seconds'].append(float(text.split(':', 1)[1]))
            except ValueError:
                pass

    def _write_sample(self):
        now = self.get_clock().now()
        row = {'type': 'sample', 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
               'sim_time_seconds': (now-self.start_time).nanoseconds/1e9,
               'state': self.state, 'battery_percent': None if self.battery is None else self.battery*100.0,
               'distance_m': self.distance, 'distance_source': 'tracked_odometry',
               'coverage_percent': self.coverage,
               'cmd_vel': {'linear_x': self.cmd.linear.x, 'linear_y': self.cmd.linear.y, 'angular_z': self.cmd.angular.z},
               'events': self.events}
        if self.odom:
            pose, frame = self._position_pose()
            twist = self.odom.twist.twist
            row['pose_frame_id'] = frame
            row['pose'] = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
                           'qx': pose.orientation.x, 'qy': pose.orientation.y, 'qz': pose.orientation.z, 'qw': pose.orientation.w}
            row['velocity'] = {'linear_x': twist.linear.x, 'linear_y': twist.linear.y, 'angular_z': twist.angular.z}
        self.file.write(json.dumps(row, separators=(',', ':'))+'\n')

    def _position_pose(self):
        """Prefer the SLAM map pose; retain the source frame if TF is not ready."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t, r = transform.transform.translation, transform.transform.rotation
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
            pose.orientation = r
            return pose, 'map'
        except Exception:
            return self.odom.pose.pose, self.odom.header.frame_id or 'odom'

    def destroy_node(self):
        if getattr(self, 'file', None) and not self.file.closed:
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
