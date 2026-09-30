from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_share = FindPackageShare('nav2_bringup')
    params = PathJoinSubstitution([FindPackageShare('robot_navigation'), 'config', 'nav2.yaml'])
    nav_launch = PathJoinSubstitution([FindPackageShare('robot_navigation'), 'launch', 'nav2_nodes.launch.py'])
    slam_launch = PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
    localization_launch = PathJoinSubstitution([nav2_share, 'launch', 'localization_launch.py'])
    slam_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch), launch_arguments={
        'use_sim_time': use_sim_time,
        'slam_params_file': PathJoinSubstitution([FindPackageShare('robot_navigation'), 'config', 'slam.yaml'])}.items(),
        condition=IfCondition(slam))
    amcl = IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch), launch_arguments={
        'use_sim_time': use_sim_time, 'map': map_file, 'params_file': params}.items(),
        condition=UnlessCondition(slam))
    navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch), launch_arguments={
        'use_sim_time': use_sim_time, 'params_file': params, 'autostart': 'true'}.items())
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('robot_navigation'), 'config', 'mvp.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}], condition=UnlessCondition(LaunchConfiguration('headless')))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        slam_node, amcl, navigation, rviz])
