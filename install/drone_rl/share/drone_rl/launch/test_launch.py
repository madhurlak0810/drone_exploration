#!/usr/bin/env python3
"""
Minimal launch file for testing drone exploration
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_drone_rl = get_package_share_directory('drone_rl')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Paths
    urdf_file = os.path.join(pkg_drone_rl, 'urdf', 'drone.urdf.xacro')
    world_file = os.path.join(pkg_drone_rl, 'worlds', 'exploration_world.world')
    config_file = os.path.join(pkg_drone_rl, 'config', 'drone_config.yaml')
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Robot state publisher
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
             world_file],
        output='screen'
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
            '-z', '0.1'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # SLAM Toolbox
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
                'resolution': 0.05,
                'max_laser_range': 10.0,
                'minimum_time_interval': 0.5
            }
        ]
    )
    
    # Q-Learning Agent
    q_learning_agent_node = ExecuteProcess(
        cmd=[os.path.join(pkg_drone_rl, '..', '..', 'bin', 'q_learning_agent')],
        output='screen',
        env={'PYTHONPATH': os.environ.get('PYTHONPATH', '')},
        additional_env={'use_sim_time': LaunchConfiguration('use_sim_time')}
    )
    
    # Static transform
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        output='screen',
        arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gazebo_server,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_drone_node,
        static_tf_node,
        slam_toolbox_node,
        q_learning_agent_node,
    ])