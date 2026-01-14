#!/usr/bin/env python3
"""
Clear-Run UGV Launch File

Launches all UGV nodes: Nav2 stack, FOD retriever, and scoop controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share directories
    pkg_share = FindPackageShare('clearrun_ugv')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    
    # Config files
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    retriever_config = PathJoinSubstitution([pkg_share, 'config', 'retriever.yaml'])
    scoop_config = PathJoinSubstitution([pkg_share, 'config', 'scoop.yaml'])
    
    # Launch arguments
    use_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    map_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file'
    )
    
    autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 stack'
    )
    
    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_share,
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )
    
    # MAVROS for UGV (ArduRover) - direct node launch
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='ugv',
        output='screen',
        parameters=[
            {'fcu_url': 'tcp://127.0.0.1:5770'},
            {'tgt_system': 2},
            {'tgt_component': 1},
            {'system_id': 2},
            {'component_id': 191},
        ],
    )
    
    # FOD Retriever node
    retriever_node = Node(
        package='clearrun_ugv',
        executable='fod_retriever',
        name='fod_retriever_node',
        namespace='ugv',
        parameters=[retriever_config],
        output='screen',
        emulate_tty=True,
    )
    
    # Scoop Controller node
    scoop_node = Node(
        package='clearrun_ugv',
        executable='scoop_controller',
        name='scoop_controller',
        namespace='ugv',
        parameters=[scoop_config],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim,
        map_file,
        autostart,
        mavros_node,
        nav2_bringup,
        retriever_node,
        scoop_node,
    ])
