import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener


STATES = ('IDLE', 'EXPLORING', 'COVERING', 'RETURNING_TO_DOCK', 'DOCKING', 'CHARGING', 'RECOVERY', 'ERROR')


class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')
        defaults = {
            'battery_capacity_percent': 100.0, 'low_battery_percent': 20.0,
            'resume_battery_percent': 95.0, 'drain_time_percent_per_second': 0.035,
            'drain_distance_percent_per_meter': 0.025, 'drain_motor_percent_per_second': 0.018,
            'charge_percent_per_second': 0.45, 'dock_radius': 0.40, 'dock_speed_threshold': 0.06,
            'stuck_speed_threshold': 0.07, 'stuck_timeout': 12.0, 'coverage_complete_percent': 98.0,
            'coverage_brush_radius': 0.19, 'frontier_min_distance': 0.35, 'dock_keepout_radius': 0.75,
            'frontier_clearance': 0.55,
            'navigation_frame': 'map', 'robot_frame': 'base_link', 'odom_frame': 'odom'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched)
        self.create_subscription(OccupancyGrid, '/coverage/grid', self._coverage_cb, latched)
        self.create_subscription(Float32, '/coverage/percent', lambda m: setattr(self, 'coverage_percent', m.data), 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 20)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(String, '/robot/control', self._control_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.state_pub = self.create_publisher(String, '/robot/state', latched)
        self.event_pub = self.create_publisher(String, '/robot/event', 20)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/coverage/grid', latched)
        self.coverage_value_pub = self.create_publisher(Float32, '/coverage/percent', 10)
        self.map = None
        self.coverage = None
        self.odom = None
        self.cmd = Twist()
        self.visited = set()
        self.coverage_dirty = True
        self.last_coverage_publish = None
        self.coverage_percent = 0.0
        self.battery = float(self.p['battery_capacity_percent'])
        self.state = 'IDLE'
        self.resume_state = 'EXPLORING'
        self.goal_handle = None
        self.goal_sequence = 0
        self.goal_in_progress = False
        self.goal_completed = False
        self.goal_failed = False
        self.goal_cell = None
        self.blacklist = set()
        self.dock_map_pose = None
        self.dock_odom_pose = None
        self.last_time = self.get_clock().now()
        self.last_odom_xy = None
        self.last_motion_pose = None
        self.motion_reference_pose = None
        self.no_progress_seconds = 0.0
        self.recovery_count = 0
        self.last_recovery_time = 0.0
        self.return_started = None
        self.dock_goal_attempts = 0
        self.last_dock_attempt = None
        self.path_history = []
        self.return_route = []
        self.total_distance = 0.0
        self.last_clearance_event = 0.0
        self._set_state('EXPLORING')
        self.create_timer(0.5, self._tick)

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))

    @staticmethod
    def _quat(yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def _set_state(self, state):
        if state not in STATES:
            state = 'ERROR'
        if self.state != state:
            self.state = state
            self.state_pub.publish(String(data=state))
            self.event_pub.publish(String(data='state:' + state))
            self.get_logger().info('Estado: %s' % state)

    def _control_cb(self, msg):
        if msg.data == 'PAUSE' and self.state in ('EXPLORING', 'COVERING'):
            self._cancel_current(lambda: self._set_state('IDLE'))
        elif msg.data == 'RESUME' and self.state == 'IDLE':
            self._set_state('EXPLORING')

    def _map_cb(self, msg):
        self.map = msg
        self.coverage_dirty = True

    def _coverage_cb(self, msg):
        self.coverage = msg

    def _odom_cb(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_odom_xy is not None:
            self.total_distance += math.hypot(xy[0]-self.last_odom_xy[0], xy[1]-self.last_odom_xy[1])
        self.last_odom_xy = xy
        self.coverage_dirty = True

    def _scan_cb(self, msg):
        # The standard ROS/Gazebo bridge has no portable Contacts -> ROS message mapping.
        # Record a near-contact proxy from the LiDAR and label it as an estimate.
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid and min(valid) < 0.17 and (abs(self.cmd.linear.x) > 0.03 or abs(self.cmd.angular.z) > 0.15):
            now = self.get_clock().now().nanoseconds/1e9
            if now-self.last_clearance_event > 2.0:
                self.last_clearance_event = now
                self._publish_event('collision_proxy_lidar_clearance')

    def _grid_index(self, x, y, grid):
        origin = grid.info.origin.position
        yaw = self._yaw(grid.info.origin.orientation)
        dx, dy = x-origin.x, y-origin.y
        gx = int((math.cos(yaw)*dx + math.sin(yaw)*dy) / grid.info.resolution)
        gy = int((-math.sin(yaw)*dx + math.cos(yaw)*dy) / grid.info.resolution)
        if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
            return gx, gy
        return None

    def _has_frontier_clearance(self, grid, gx, gy):
        """Reject goal cells too close to mapped obstacles for this footprint."""
        radius = int(math.ceil(self.p['frontier_clearance'] / grid.info.resolution))
        radius_sq = self.p['frontier_clearance'] ** 2
        width, height = grid.info.width, grid.info.height
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                if dx*dx + dy*dy > radius_sq/(grid.info.resolution**2):
                    continue
                nx, ny = gx+dx, gy+dy
                if not (0 <= nx < width and 0 <= ny < height):
                    return False
                if grid.data[ny*width+nx] >= 65:
                    return False
        return True

    def _publish_coverage(self):
        if self.map is None or self.odom is None:
            return
        # OccupancyGrid is in map, while filtered odometry is in odom. Apply TF
        # before indexing so SLAM's map->odom correction does not shift coverage.
        map_pose = self._map_pose()
        if map_pose is None:
            return
        center = self._grid_index(map_pose[0], map_pose[1], self.map)
        if center:
            radius = max(1, int(self.p['coverage_brush_radius'] / self.map.info.resolution))
            cx, cy = center
            for gy in range(max(0, cy-radius), min(self.map.info.height, cy+radius+1)):
                for gx in range(max(0, cx-radius), min(self.map.info.width, cx+radius+1)):
                    if (gx-cx)**2 + (gy-cy)**2 > radius**2:
                        continue
                    idx = gy*self.map.info.width+gx
                    if self.map.data[idx] >= 0 and self.map.data[idx] < 25:
                        self.visited.add(idx)
        grid = OccupancyGrid()
        grid.header = self.map.header
        grid.info = self.map.info
        data = []
        known_free = 0
        visited = 0
        for idx, value in enumerate(self.map.data):
            if value >= 0 and value < 25:
                known_free += 1
                was_visited = idx in self.visited
                data.append(100 if was_visited else 0)
                visited += int(was_visited)
            else:
                data.append(-1)
        grid.data = data
        self.coverage_pub.publish(grid)
        pct = 100.0 * visited / max(1, known_free)
        self.coverage_percent = pct
        self.coverage_value_pub.publish(Float32(data=pct))
        self.coverage_dirty = False
        self.last_coverage_publish = self.get_clock().now()

    def _map_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(self.p['navigation_frame'], self.p['robot_frame'], rclpy.time.Time())
            t = tr.transform.translation
            yaw = self._yaw(tr.transform.rotation)
            return t.x, t.y, yaw
        except Exception:
            return None

    def _battery_tick(self, dt):
        speed = math.hypot(self.cmd.linear.x, self.cmd.linear.y) + abs(self.cmd.angular.z) * 0.25
        if self.state == 'CHARGING':
            self.battery = min(100.0, self.battery + self.p['charge_percent_per_second']*dt)
        else:
            distance_delta = 0.0
            if self.odom is not None:
                p = self.odom.pose.pose.position
                xy = (p.x, p.y)
                if getattr(self, '_battery_xy', None) is not None:
                    distance_delta = math.hypot(xy[0]-self._battery_xy[0], xy[1]-self._battery_xy[1])
                self._battery_xy = xy
            drain = (self.p['drain_time_percent_per_second']*dt +
                     self.p['drain_distance_percent_per_meter']*distance_delta +
                     self.p['drain_motor_percent_per_second']*min(1.0, speed)*dt)
            self.battery = max(0.0, self.battery-drain)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float('nan')
        msg.percentage = self.battery/100.0
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.state == 'CHARGING' else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(msg)

    def _tick(self):
        now = self.get_clock().now()
        dt = max(0.0, (now-self.last_time).nanoseconds/1e9)
        self.last_time = now
        self._battery_tick(dt)
        pose = self._map_pose()
        # Rebuilding and publishing the whole grid at odometry rate starves this
        # executor. Publish only from the 2 Hz autonomy loop, and skip clean grids.
        if self.coverage_dirty and (self.last_coverage_publish is None or
                (now-self.last_coverage_publish).nanoseconds >= 500_000_000):
            self._publish_coverage()
        if self.odom is not None and self.dock_odom_pose is None and pose is not None:
            p = self.odom.pose.pose
            self.dock_odom_pose = (p.position.x, p.position.y, self._yaw(p.orientation))
            self.dock_map_pose = pose
            self.get_logger().info('Pose da base capturada em odom: %.2f, %.2f' %
                                   (self.dock_odom_pose[0], self.dock_odom_pose[1]))
        self._record_path_pose(pose)
        if self.state not in ('RETURNING_TO_DOCK', 'DOCKING', 'CHARGING', 'ERROR') and self.battery <= self.p['low_battery_percent']:
            self.resume_state = self.state if self.state in ('EXPLORING', 'COVERING') else 'EXPLORING'
            self.return_started = now
            self._set_state('RETURNING_TO_DOCK')
            self._publish_event('battery_low')
            self._prepare_return_route(pose)
            self._cancel_current(self._send_dock_goal)
            return
        self._check_stuck(pose, dt)
        if self.state == 'CHARGING':
            if self.battery >= self.p['resume_battery_percent']:
                self._publish_event('charging_complete')
                self._set_state(self.resume_state)
                self.goal_completed = False
            return
        if self.state in ('RETURNING_TO_DOCK', 'DOCKING') and self.odom is not None and self.dock_odom_pose:
            speed = math.hypot(self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y) if self.odom else 0.0
            # The charging target is a fixed landmark in the SLAM map. Odometry
            # can drift, so use the map pose for the docking proximity check too.
            dist = (math.hypot(pose[0]-self.dock_map_pose[0], pose[1]-self.dock_map_pose[1])
                    if pose is not None and self.dock_map_pose is not None else float('inf'))
            if dist <= self.p['dock_radius'] and speed <= self.p['dock_speed_threshold']:
                self._set_state('CHARGING')
                if self.return_started:
                    secs = (now-self.return_started).nanoseconds/1e9
                    self._publish_event('return_to_dock_seconds:%.3f' % secs)
                return
            if self.goal_in_progress:
                return
            if self.goal_failed:
                self.goal_failed = False
                self.dock_goal_attempts += 1
                self._publish_event('dock_navigation_retry:%d' % self.dock_goal_attempts)
                if self.dock_goal_attempts >= 3:
                    if len(self.return_route) > 1:
                        self.return_route.pop(0)
                        self.dock_goal_attempts = 0
                        self._publish_event('dock_waypoint_skipped')
                    else:
                        self._set_state('ERROR')
                        self._publish_event('dock_navigation_failed')
                        return
                self.last_dock_attempt = now
            if self.goal_completed:
                self.goal_completed = False
                if len(self.return_route) > 1:
                    self.return_route.pop(0)
                    self.dock_goal_attempts = 0
                    self.last_dock_attempt = None
                    self._send_dock_goal()
                    return
                self._set_state('DOCKING')
                self.last_dock_attempt = now
            if self.last_dock_attempt is None or (now-self.last_dock_attempt).nanoseconds/1e9 >= 2.0:
                self._send_dock_goal()
            return
        if self.state == 'RECOVERY':
            if now.nanoseconds/1e9 - self.last_recovery_time > 3.0:
                self._set_state(self.resume_state if self.resume_state in ('EXPLORING','COVERING') else 'EXPLORING')
            return
        if self.state not in ('EXPLORING', 'COVERING') or self.goal_in_progress or pose is None:
            return
        if self.goal_completed:
            self.goal_completed = False
            self.goal_cell = None
        if self.goal_failed:
            self.goal_failed = False
            if self.goal_cell is not None:
                self.blacklist.add(self.goal_cell)
        goal = None
        if self.state == 'EXPLORING':
            goal = self._select_frontier(pose)
            if goal is None:
                self._set_state('COVERING')
        if goal is None and self.state == 'COVERING':
            goal = self._select_unvisited(pose)
            if goal is None:
                if self.coverage_percent >= self.p['coverage_complete_percent']:
                    self._set_state('IDLE')
                    self._publish_event('coverage_complete')
                return
        if goal:
            cell, x, y = goal
            self.goal_cell = cell
            self._send_goal(x, y, pose[2])

    def _check_stuck(self, pose, dt):
        if self.odom is None:
            return
        moving_command = math.hypot(self.cmd.linear.x, self.cmd.linear.y) + abs(self.cmd.angular.z)*0.2
        current = (pose[0], pose[1], pose[2]) if pose else None
        progressed = False
        if current and self.motion_reference_pose:
            # Compare against the position at the start of the no-progress
            # window. Comparing consecutive samples lets small SLAM jitter
            # reset the timer forever while the chassis is physically stuck.
            progressed = (math.hypot(current[0]-self.motion_reference_pose[0],
                                     current[1]-self.motion_reference_pose[1]) > 0.10 or
                          abs(math.atan2(math.sin(current[2]-self.motion_reference_pose[2]),
                                         math.cos(current[2]-self.motion_reference_pose[2]))) > 0.15)
        if current:
            self.last_motion_pose = current
        if moving_command > self.p['stuck_speed_threshold'] and not progressed:
            if self.motion_reference_pose is None:
                self.motion_reference_pose = current
            self.no_progress_seconds += dt
        else:
            self.no_progress_seconds = 0.0
            self.motion_reference_pose = current
        if self.no_progress_seconds >= self.p['stuck_timeout'] and self.state in ('EXPLORING','COVERING'):
            self.recovery_count += 1
            self.last_recovery_time = self.get_clock().now().nanoseconds/1e9
            self.resume_state = self.state
            self._set_state('RECOVERY')
            self._publish_event('stuck_detected')
            self._publish_event('recovery_attempt:%d' % self.recovery_count)
            self.no_progress_seconds = 0.0
            self.motion_reference_pose = current
            if self.goal_cell is not None:
                self.blacklist.add(self.goal_cell)

    def _publish_event(self, data):
        self.event_pub.publish(String(data=data))

    def _record_path_pose(self, map_pose):
        if map_pose is None:
            return
        sample = map_pose
        if not self.path_history:
            self.path_history.append(sample)
        elif math.hypot(sample[0]-self.path_history[-1][0], sample[1]-self.path_history[-1][1]) >= 0.35:
            self.path_history.append(sample)

    def _prepare_return_route(self, map_pose):
        if map_pose is None or self.dock_map_pose is None:
            self.return_route = [self.dock_map_pose] if self.dock_map_pose else []
            return
        # Follow recently traversed map poses in reverse. Short segments
        # remain inside the observed corridor while SLAM is still extending map.
        # Keep breadcrumbs in map coordinates so later map->odom corrections do
        # not move the targets while the robot is driving back.
        self.return_route = [point for point in reversed(self.path_history)
                             if math.hypot(point[0]-map_pose[0], point[1]-map_pose[1]) >= 0.30]
        if not self.return_route or math.hypot(
                self.return_route[-1][0]-self.dock_map_pose[0],
                self.return_route[-1][1]-self.dock_map_pose[1]) > 0.20:
            self.return_route.append(self.dock_map_pose)
        self.dock_goal_attempts = 0

    def _send_goal(self, x, y, yaw, frame_id=None):
        if not self.action.server_is_ready():
            self.action.wait_for_server(timeout_sec=0.05)
            if not self.action.server_is_ready():
                return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id or self.p['navigation_frame']
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = self._quat(yaw)
        self.get_logger().info('Enviando objetivo Nav2 em %s: (%.2f, %.2f), estado=%s' %
                               (goal.pose.header.frame_id, x, y, self.state))
        self.goal_in_progress = True
        self.goal_completed = False
        self.goal_failed = False
        self.goal_sequence += 1
        sequence = self.goal_sequence
        future = self.action.send_goal_async(goal)
        future.add_done_callback(lambda result, seq=sequence: self._goal_response(result, seq))

    def _goal_response(self, future, sequence):
        try:
            handle = future.result()
            if sequence != self.goal_sequence:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.goal_in_progress = False
                self.goal_failed = True
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda result, seq=sequence: self._goal_result(result, seq))
        except Exception as exc:
            if sequence != self.goal_sequence:
                return
            self.goal_in_progress = False
            self.goal_failed = True
            self.get_logger().error('Falha enviando objetivo: %s' % exc)

    def _goal_result(self, future, sequence):
        if sequence != self.goal_sequence:
            return
        try:
            result = future.result()
            self.goal_in_progress = False
            self.goal_handle = None
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.goal_completed = True
                if self.goal_cell is not None:
                    self.blacklist.add(self.goal_cell)
            else:
                self.goal_failed = True
                self._publish_event('navigation_goal_failed')
        except Exception as exc:
            self.goal_in_progress = False
            self.goal_failed = True
            self.get_logger().warning('Resultado de navegação: %s' % exc)

    def _cancel_current(self, then=None):
        # Invalidate callbacks from the goal being canceled before the replacement
        # goal is sent. Otherwise a late canceled result can mark the new goal failed.
        self.goal_sequence += 1
        if self.goal_handle is not None and self.goal_in_progress:
            future = self.goal_handle.cancel_goal_async()
            def done(_):
                self.goal_in_progress = False
                self.goal_handle = None
                if then:
                    then()
            future.add_done_callback(done)
        else:
            self.goal_in_progress = False
            self.goal_handle = None
            if then:
                then()

    def _send_dock_goal(self):
        if self.return_route:
            x, y, yaw = self.return_route[0]
            self.last_dock_attempt = self.get_clock().now()
            self._send_goal(x, y, yaw, self.p['navigation_frame'])
        elif self.dock_map_pose:
            x, y, yaw = self.dock_map_pose
            self.last_dock_attempt = self.get_clock().now()
            self._send_goal(x, y, yaw, self.p['navigation_frame'])
        else:
            self._set_state('ERROR')
            self._publish_event('dock_pose_unavailable')

    def _select_frontier(self, robot_pose):
        grid = self.map
        if grid is None or not grid.data:
            return None
        width, height = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        yaw0 = self._yaw(grid.info.origin.orientation)
        candidates = []
        for gy in range(1, height-1, 2):
            row = gy*width
            for gx in range(1, width-1, 2):
                index = row+gx
                if grid.data[index] < 0 or grid.data[index] >= 25 or index in self.blacklist:
                    continue
                unknown = any(grid.data[(gy+dy)*width+gx+dx] < 0 for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)))
                if not unknown:
                    continue
                lx, ly = (gx+0.5)*res, (gy+0.5)*res
                x = ox+math.cos(yaw0)*lx-math.sin(yaw0)*ly
                y = oy+math.sin(yaw0)*lx+math.cos(yaw0)*ly
                d = math.hypot(x-robot_pose[0],y-robot_pose[1])
                if self.dock_map_pose and math.hypot(x-self.dock_map_pose[0], y-self.dock_map_pose[1]) < self.p['dock_keepout_radius']:
                    continue
                if d >= self.p['frontier_min_distance']:
                    candidates.append((d,index,x,y))
        for d, index, x, y in sorted(candidates):
            gx, gy = index % width, index // width
            if self._has_frontier_clearance(grid, gx, gy):
                return index, x, y
        return None

    def _select_unvisited(self, robot_pose):
        grid = self.coverage
        if grid is None or not grid.data:
            return None
        width, height = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        yaw0 = self._yaw(grid.info.origin.orientation)
        candidates = []
        for gy in range(2, height-2, 5):
            for gx in range(2, width-2, 5):
                idx=gy*width+gx
                if grid.data[idx] != 0 or idx in self.blacklist:
                    continue
                lx, ly = (gx+0.5)*res, (gy+0.5)*res
                x=ox+math.cos(yaw0)*lx-math.sin(yaw0)*ly
                y=oy+math.sin(yaw0)*lx+math.cos(yaw0)*ly
                d=math.hypot(x-robot_pose[0],y-robot_pose[1])
                if self.dock_map_pose and math.hypot(x-self.dock_map_pose[0], y-self.dock_map_pose[1]) < self.p['dock_keepout_radius']:
                    continue
                candidates.append((d,idx,x,y))
        for d, idx, x, y in sorted(candidates):
            if self._has_frontier_clearance(grid, idx % width, idx // width):
                return idx, x, y
        return None


def main():
    rclpy.init()
    node = AutonomyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
