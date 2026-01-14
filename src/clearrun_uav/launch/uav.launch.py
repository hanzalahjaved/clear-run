#!/usr/bin/env python3
"""
Clear-Run UAV Launch File

Launches all UAV nodes: detection, visual servo, and MAVROS interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('clearrun_uav')
    
    # Config files
    detection_config = PathJoinSubstitution([pkg_share, 'config', 'detection.yaml'])
    servo_config = PathJoinSubstitution([pkg_share, 'config', 'visual_servo.yaml'])
    
    # Launch arguments
    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode'
    )
    
    fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='tcp://127.0.0.1:5760',
        description='FCU connection URL (SITL TCP port)'
    )
    
    # MAVROS node - direct launch (ROS 2 Jazzy)
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='uav',
        output='screen',
        parameters=[
            {'fcu_url': LaunchConfiguration('fcu_url')},
            {'tgt_system': 1},
            {'tgt_component': 1},
            {'system_id': 1},
            {'component_id': 191},
        ],
    )
    
    # Detection node
    detection_node = Node(
        package='clearrun_uav',
        executable='detection_node',
        name='detection_node',
        namespace='uav',
        parameters=[detection_config],
        output='screen',
        emulate_tty=True,
    )
    
    # Visual servo node
    visual_servo_node = Node(
        package='clearrun_uav',
        executable='visual_servo',
        name='visual_servo_node',
        namespace='uav',
        parameters=[servo_config],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim,
        fcu_url,
        mavros_node,
        detection_node,
        visual_servo_node,
    ])
