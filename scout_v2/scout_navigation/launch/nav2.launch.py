from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params')
    default_bt_xml = LaunchConfiguration('default_bt_xml_filename')

    # Parameters and BT file configuration
    params_pkg = FindPackageShare('scout_navigation')
    params = PathJoinSubstitution([params_pkg, 'config', 'mppi_nav2.yaml'])
    default_bt_pkg = FindPackageShare('metalbot')
    default_bt = PathJoinSubstitution([default_bt_pkg, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'])

    # RViz configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory('scout_navigation'),
        'rviz',
        'test.rviz')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml,
        'autostart': autostart,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    # Launch Description
    ld = LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params', default_value=[params], description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('default_bt_xml_filename', default_value=default_bt, description='Full path to the behavior tree xml file to use'),

        # Nav2 Nodes
        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[configured_params]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[configured_params]),
        Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server', output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[configured_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[configured_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen', parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart}, {'node_names': lifecycle_nodes}]),

        # RViz Node
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config_dir], output='screen'),
    ])

    return ld
