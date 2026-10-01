from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    navigation_enabled = LaunchConfiguration('navigation_enabled')
    autonomy_enabled = LaunchConfiguration('autonomy_enabled')
    metrics_enabled = LaunchConfiguration('metrics_enabled')
    localization_enabled = LaunchConfiguration('localization_enabled')
    battery_low_percent = LaunchConfiguration('battery_low_percent')
    battery_resume_percent = LaunchConfiguration('battery_resume_percent')
    world = LaunchConfiguration('world')
    sim_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    nav_launch = PathJoinSubstitution([FindPackageShare('robot_navigation'), 'launch', 'navigation.launch.py'])

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'gz_args': ["-r -v 3 \"", world, "\""]}.items(),
        condition=UnlessCondition(headless))
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'gz_args': ["-s -r -v 3 \"", world, "\""]}.items(),
        condition=IfCondition(headless))

    robot_description = Command([
        'xacro "', PathJoinSubstitution([FindPackageShare('robot_description'), 'urdf', 'aspirador.urdf.xacro']), '"'])
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen', parameters=[{
            'robot_description': robot_description, 'use_sim_time': use_sim_time}])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        output='screen', arguments=[
            '/world/casa/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/wheel/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ], remappings=[('/world/casa/clock', '/clock'), ('/cmd_vel', '/cmd_vel_gz')],
        parameters=[{'use_sim_time': use_sim_time}])
    # Gazebo Harmonic's TrackedVehicle uses the opposite yaw sign from ROS Twist.
    # Keep the ROS /cmd_vel convention standard and adapt only at the simulator boundary.
    cmd_vel_adapter = Node(package='robot_simulation', executable='cmd_vel_adapter.py',
        name='cmd_vel_adapter', output='screen')
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[PathJoinSubstitution([FindPackageShare('robot_navigation'), 'config', 'ekf.yaml']), {'use_sim_time': use_sim_time}],
        condition=IfCondition(localization_enabled))

    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch), launch_arguments={
        'use_sim_time': use_sim_time, 'slam': slam, 'map': map_file,
        'headless': headless}.items(), condition=IfCondition(navigation_enabled))
    autonomy = Node(package='robot_autonomy', executable='autonomy_manager', name='autonomy_manager',
        output='screen', parameters=[PathJoinSubstitution([FindPackageShare('robot_autonomy'), 'config', 'autonomy.yaml']), {
            'use_sim_time': use_sim_time,
            'low_battery_percent': ParameterValue(battery_low_percent, value_type=float),
            'resume_battery_percent': ParameterValue(battery_resume_percent, value_type=float)}],
        condition=IfCondition(autonomy_enabled))
    metrics = Node(package='robot_metrics', executable='metrics_recorder', name='metrics_recorder',
        output='screen', parameters=[{'use_sim_time': use_sim_time}], condition=IfCondition(metrics_enabled))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('navigation_enabled', default_value='true'),
        DeclareLaunchArgument('autonomy_enabled', default_value='true'),
        DeclareLaunchArgument('metrics_enabled', default_value='true'),
        DeclareLaunchArgument('localization_enabled', default_value='true'),
        DeclareLaunchArgument('battery_low_percent', default_value='20.0'),
        DeclareLaunchArgument('battery_resume_percent', default_value='95.0'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [FindPackageShare('robot_description'), 'worlds', 'casa.sdf'])),
        gz_gui, gz_headless, state_publisher,
        TimerAction(period=4.0, actions=[bridge, cmd_vel_adapter, ekf]),
        TimerAction(period=10.0, actions=[nav]),
        TimerAction(period=18.0, actions=[autonomy, metrics]),
    ])
