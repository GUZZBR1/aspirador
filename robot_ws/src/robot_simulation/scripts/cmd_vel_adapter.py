#!/usr/bin/env python3
"""Keep a stable ROS command interface at the Gazebo transport boundary."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_adapter')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_gz', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self._forward, 10)

    def _forward(self, message):
        adapted = Twist()
        adapted.linear.x = message.linear.x
        adapted.linear.y = message.linear.y
        adapted.linear.z = message.linear.z
        adapted.angular.x = message.angular.x
        adapted.angular.y = message.angular.y
        # The current Harmonic TrackedVehicle system and its odometry agree with
        # the ROS yaw convention for this model. Do not invert here: doing so
        # makes Nav2 rotate away from the path heading.
        adapted.angular.z = message.angular.z
        self.publisher.publish(adapted)


def main():
    rclpy.init()
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
