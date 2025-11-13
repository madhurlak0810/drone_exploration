#!/usr/bin/env python3
"""
Launch file for autonomous drone exploration simulation
Integrates Gazebo, RTAB-Map SLAM, and Q-Learning agent
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_drone_rl = get_package_share_directory('drone_rl')
    # pkg_rtabmap_launch = get_package_share_directory('rtabmap_launch')  # Not needed
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')  # Not needed
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_drone_rl, 'worlds', 'exploration_world.world'),
        description='Full path to world file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='slam_toolbox',
        description='SLAM method: slam_toolbox or rtabmap'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Paths
    urdf_file = os.path.join(pkg_drone_rl, 'urdf', 'drone.urdf.xacro')
    config_file = os.path.join(pkg_drone_rl, 'config', 'drone_config.yaml')
    
    # Robot state publisher - read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description}
        ]
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', 
             '--verbose',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen'
    )
    
    # Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )
    
    # Spawn drone in Gazebo
    spawn_drone_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drone',
        output='screen',
        arguments=[
            '-entity', 'drone',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # SLAM Toolbox (alternative to RTAB-Map)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                
                # SLAM Toolbox parameters
                'resolution': 0.05,
                'max_laser_range': 10.0,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
                
                # Loop closure
                'enable_interactive_mode': False,
                'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                'ceres_preconditioner': 'SCHUR_JACOBI',
                'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                
                # Scan matcher
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.2,
                'minimum_travel_heading': 0.2,
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                'link_match_minimum_response_fine': 0.1,
                'link_scan_maximum_distance': 1.5,
                'loop_search_maximum_distance': 3.0,
                'do_loop_closing': True,
                'loop_match_minimum_chain_size': 10,
                'loop_match_maximum_variance_coarse': 3.0,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_minimum_response_fine': 0.45
            }
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # Q-Learning Agent
    q_learning_agent_node = Node(
        package='drone_rl',
        executable='q_learning_agent',
        name='q_learning_agent',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Environment Interface
    env_interface_node = Node(
        package='drone_rl',
        executable='environment_interface',
        name='environment_interface',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Exploration Metrics
    metrics_node = Node(
        package='drone_rl',
        executable='exploration_metrics',
        name='exploration_metrics',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # RViz for general visualization
    rviz_config = os.path.join(pkg_drone_rl, 'config', 'drone_exploration.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Static transforms (if needed)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        output='screen',
        arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    return LaunchDescription([
        # Arguments
        world_arg,
        use_sim_time_arg,
        headless_arg,
        slam_arg,
        rviz_arg,
        
        # Core simulation
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_drone_node,
        static_tf_node,
        
        # SLAM
        slam_toolbox_node,
        
        # RL Components  
        q_learning_agent_node,
        env_interface_node,
        metrics_node,
        
        # Visualization
        rviz_node,
    ])