import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    declare_enable_tb3_sim = DeclareLaunchArgument('enable_tb3_sim', default_value='false')
    declare_enable_rviz = DeclareLaunchArgument('enable_rviz', default_value='false')
    declare_joy_node_variant = DeclareLaunchArgument(
        'joy_node',
        default_value='joy',
        description="Which joy node to use: 'wsl' (evdev_joy_node), 'joy' (standard joy_node) or 'none' (no joy node).")
    declare_teleop_variant = DeclareLaunchArgument(
        "teleop",
        default_value="joy",
        description="Which teleop node to use: 'joy' (teleop_twist_joy), 'key' (teleop_twist_keyboard) or none (no teleop).")
    declare_logger = DeclareLaunchArgument(
        "log_level",
        default_value=["warn"],
        description="Logging level")

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_safety_filter = get_package_share_directory('safety_filter_mpc')

    enable_tb3_sim_cfg = LaunchConfiguration('enable_tb3_sim')

    # --- Turtlebot Simulation ---
    start_gazebo_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ),
        condition=IfCondition(enable_tb3_sim_cfg)
    )

    # --- SLAM ---
    slam_toolbox_params_file = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')
    start_slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_params_file,
            {'use_sim_time': enable_tb3_sim_cfg}],
    )

    # --- TELEOP ---
    teleop_variant = LaunchConfiguration('teleop')
    joy_node_variant = LaunchConfiguration('joy_node')
    joystick_config_file = os.path.join(pkg_safety_filter, 'config', 'joy_config.yaml')
    start_wsl_joy_bridge_cmd = Node(
        package='wsl_joystick_bridge',
        executable='evdev_joy_node',
        name='evdev_joy_node',
        output='screen',
        parameters=[
            joystick_config_file,
            {'use_sim_time': enable_tb3_sim_cfg}],
        condition=IfCondition(PythonExpression(
            ["'", joy_node_variant, "' == 'wsl' and '", teleop_variant, "' == 'joy' "]))
    )

    start_joy_node_cmd = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            joystick_config_file,
            {'use_sim_time': enable_tb3_sim_cfg}],
        condition=IfCondition(PythonExpression(
            ["'", joy_node_variant, "' == 'joy' and '", teleop_variant, "' == 'joy' "]))
    )

    start_teleop_joy_cmd = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel_unsafe')],
        parameters=[
            joystick_config_file,
            {'use_sim_time': enable_tb3_sim_cfg}],
        condition=IfCondition(PythonExpression(
            ["'", teleop_variant, "' == 'joy' "]))
    )

    start_teleop_keyboard_cmd = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/cmd_vel_unsafe')],
        parameters=[
            {'use_sim_time': enable_tb3_sim_cfg}],
        condition=IfCondition(PythonExpression(
            ["'", joy_node_variant, "' == 'key' "]))
    )

    # --- Safety Filter ---
    sf_scout2_config_file = os.path.join(pkg_safety_filter, 'config', 'scout2_config.yaml')
    sf_tb3_config_file = os.path.join(pkg_safety_filter, 'config', 'turtlebot_waffle_config.yaml')
    safety_filter_config = PythonExpression([
        f"'{sf_tb3_config_file}'",
        " if '", LaunchConfiguration('enable_tb3_sim'), "' == 'true' else ",
        f"'{sf_scout2_config_file}'"
    ])
    
    start_sf_cmd = Node(
        package='safety_filter_mpc',
        executable='safety_filter_mpc_node',
        name='safety_filter_mpc',
        output='screen',
        remappings=[
            ('/cmd_vel_safe', '/cmd_vel')],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            safety_filter_config,
            {'use_sim_time': enable_tb3_sim_cfg}],
    )

    # --- Vizualisation ---
    rviz_config_file = os.path.join(pkg_safety_filter, 'config', 'nav2_safe_teleop.rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config_file],
        parameters=[
            {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_enable_tb3_sim)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_joy_node_variant)
    ld.add_action(declare_teleop_variant)
    ld.add_action(declare_logger)
    ld.add_action(start_gazebo_sim_cmd)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_sf_cmd)
    ld.add_action(start_wsl_joy_bridge_cmd)
    ld.add_action(start_joy_node_cmd)
    ld.add_action(start_teleop_joy_cmd)
    ld.add_action(start_teleop_keyboard_cmd)
    ld.add_action(start_rviz_cmd)

    return ld