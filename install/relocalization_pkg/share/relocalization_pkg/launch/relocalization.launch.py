import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Parameters
    rviz_file_name = 'relocalization.rviz'
    map_file_name = 'indoor.yaml'
    
    # Configaration variables
    pkg_share = FindPackageShare('relocalization_pkg').find('relocalization_pkg')
    nav2_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    rviz_config_file = os.path.join(pkg_share, 'rviz', rviz_file_name)
    static_map_path = os.path.join(pkg_share, 'maps', map_file_name)
    nav2_bt_path = FindPackageShare('nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_composition = LaunchConfiguration('use_composition')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    
    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RViz')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='True',
        description='Whether to apply namespace to navigation nodes')
    
    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether run SLAM')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load')
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        name='use_composition',
        default_value='False',
        description='Whether to use a component container')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_bt_xml_cmd = DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=behavior_tree_xml_path,
        description='Full path to the behavior tree xml file to use')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')
    
    # Nodes to launch
    ekf_odometry_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'ekf_odometry.yaml'),
                {'use_sim_time': use_sim_time}],
        )
    
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': slam,
                            'map': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'use_composition': use_composition,
                            'params_file': params_file,
                            'default_bt_xml_filename': default_bt_xml_filename,
                            'autostart': autostart}.items())
    
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}],
        arguments=['-d', rviz_config_file])  
     
    acquire_query_node = Node(
        package='relocalization_pkg',
        executable='acquire_query_node',
        name='acquire_query_node',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}])
    
    pose_estimation_node = Node(
        package='relocalization_pkg',
        executable='pose_estimation_node_v2',
        name='pose_estimation_node',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}])
    
    reloc_eval_node = Node(
        package='relocalization_pkg',
        executable='reloc_eval',
        name='reloc_eval',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}])
    
    converge_to_pose_node = Node(
        package='relocalization_pkg',
        executable='converge_to_pose_node',
        name='converge_to_pose_node',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}])
    
    ld = LaunchDescription()
    
    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add nodes
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(ekf_odometry_node)
    ld.add_action(start_rviz_cmd)

    
    return ld