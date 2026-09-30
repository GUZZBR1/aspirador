"""Run the Nav2 servers needed by this MVP without unused servers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    managed = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'velocity_smoother', 'collision_monitor',
        'bt_navigator', 'waypoint_follower',
    ]
    nodes = [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params], remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params], remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[params], remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': autostart, 'node_names': managed,
                          'use_sim_time': use_sim_time, 'service_timeout': 60.0}]),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('robot_navigation'), 'config', 'nav2.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', use_sim_time),
        *nodes,
    ])
