"""Isolated physical probes; launch once per scenario in a fresh simulator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    output_dir = LaunchConfiguration('output_dir')
    headless = LaunchConfiguration('headless')
    bed_enabled = PythonExpression(["'", scenario, "' == 'bed'"])
    bed_map = PathJoinSubstitution([
        FindPackageShare('robot_description'), 'maps', 'bed_test_map.yaml'])
    world = PathJoinSubstitution([
        FindPackageShare('robot_description'), 'worlds',
        PythonExpression(["'", scenario, "_test.sdf'"]),
    ])
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_simulation'), 'launch', 'mvp.launch.py'])),
        launch_arguments={
            'world': world,
            'headless': headless,
            'slam': 'false',
            'map': bed_map,
            'navigation_enabled': bed_enabled,
            'autonomy_enabled': 'false',
            'metrics_enabled': 'false',
            'localization_enabled': bed_enabled,
            'battery_low_percent': '0.0',
        }.items())
    # Both fixtures name their SDF world 'casa' to share the existing clock
    # bridge. Keep ground truth off /tf so it cannot contaminate SLAM or EKF.
    pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='physics_pose_bridge', output='screen',
        arguments=['/model/aspirador/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/aspirador/pose', '/physics/pose_info')])
    runner = Node(
        package='robot_tests', executable='physics_runner', output='screen',
        parameters=[{
            'scenario': scenario,
            'output_dir': output_dir,
            'startup_timeout': ParameterValue(
                LaunchConfiguration('startup_timeout'), value_type=float),
            'test_timeout': ParameterValue(
                LaunchConfiguration('test_timeout'), value_type=float),
            'use_sim_time': True,
        }])
    exit_on_finish = RegisterEventHandler(
        OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='threshold',
                              description='threshold or bed; run in separate launches'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value='results'),
        DeclareLaunchArgument('startup_timeout', default_value='90.0'),
        DeclareLaunchArgument('test_timeout', default_value='120.0'),
        sim,
        TimerAction(period=4.0, actions=[pose_bridge]),
        exit_on_finish,
        TimerAction(period=6.0, actions=[runner]),
    ])
