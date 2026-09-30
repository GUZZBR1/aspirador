from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    output_dir = LaunchConfiguration('output_dir')
    scenario_limit = LaunchConfiguration('scenario_limit')
    scenario_name = LaunchConfiguration('scenario_name')
    run_control_tests = LaunchConfiguration('run_control_tests')
    warmup_seconds = LaunchConfiguration('warmup_seconds')
    goal_timeout = LaunchConfiguration('goal_timeout')
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('robot_simulation'), 'launch', 'mvp.launch.py'])),
        launch_arguments={'headless': 'true', 'slam': 'true', 'navigation_enabled': 'true',
                          'autonomy_enabled': 'true', 'metrics_enabled': 'true',
                          'battery_low_percent': '0.0'}.items())
    runner = Node(package='robot_tests', executable='mobility_runner', output='screen',
                  parameters=[{'output_dir': output_dir, 'goal_timeout': goal_timeout,
                               'scenario_limit': scenario_limit, 'scenario_name': scenario_name,
                               'run_control_tests': run_control_tests}])
    exit_when_finished = RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='results'),
        DeclareLaunchArgument('scenario_limit', default_value='8'),
        DeclareLaunchArgument('scenario_name', default_value=''),
        DeclareLaunchArgument('run_control_tests', default_value='true'),
        DeclareLaunchArgument('warmup_seconds', default_value='120.0'),
        DeclareLaunchArgument('goal_timeout', default_value='45.0'),
        sim,
        exit_when_finished,
        # Let autonomous SLAM explore and connect the test regions before
        # deterministic scenario goals start. The runner pauses autonomy first.
        TimerAction(period=warmup_seconds, actions=[runner]),
    ])
