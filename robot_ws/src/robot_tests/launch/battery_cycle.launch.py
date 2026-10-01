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
    headless = LaunchConfiguration('headless')
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('robot_simulation'), 'launch', 'mvp.launch.py'])),
        launch_arguments={'headless': headless, 'slam': 'true', 'navigation_enabled': 'true',
                          'autonomy_enabled': 'true', 'metrics_enabled': 'true',
                          'battery_low_percent': '96.0', 'battery_resume_percent': '97.0'}.items())
    runner = Node(package='robot_tests', executable='battery_runner', output='screen',
                  parameters=[{'output_dir': output_dir, 'timeout_seconds': 300.0,
                               'resume_battery_percent': 97.0}])
    shutdown = RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='results'),
        DeclareLaunchArgument('headless', default_value='false'), sim, shutdown,
        TimerAction(period=2.0, actions=[runner]),
    ])
