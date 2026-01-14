#!/usr/bin/env python3
"""
Clear-Run Full System Launch

Launches the complete Clear-Run system including:
- UAV: Detection node, Visual servo, MAVROS
- UGV: Nav2 stack, FOD retriever, Scoop controller
- Visualization: RViz with custom config
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =========================================================================
    # Package Shares
    # =========================================================================
    bringup_share = FindPackageShare('clearrun_bringup')
    uav_share = FindPackageShare('clearrun_uav')
    ugv_share = FindPackageShare('clearrun_ugv')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    launch_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    uav_fcu_url = DeclareLaunchArgument(
        'uav_fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='UAV FCU connection URL'
    )
    
    ugv_fcu_url = DeclareLaunchArgument(
        'ugv_fcu_url',
        default_value='udp://:14550@127.0.0.1:14555',
        description='UGV FCU connection URL'
    )
    
    map_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file for UGV navigation'
    )
    
    # =========================================================================
    # UAV Launch Group
    # =========================================================================
    uav_group = GroupAction([
        LogInfo(msg='Launching UAV nodes...'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    uav_share,
                    'launch',
                    'uav.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim': LaunchConfiguration('use_sim_time'),
                'fcu_url': LaunchConfiguration('uav_fcu_url'),
            }.items()
        ),
    ])
    
    # =========================================================================
    # UGV Launch Group (delayed start)
    # =========================================================================
    ugv_group = TimerAction(
        period=5.0,  # Delay UGV startup by 5 seconds
        actions=[
            GroupAction([
                LogInfo(msg='Launching UGV nodes...'),
                
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            ugv_share,
                            'launch',
                            'ugv.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'map': LaunchConfiguration('map'),
                    }.items()
                ),
            ])
        ]
    )
    
    # =========================================================================
    # RViz Visualization
    # =========================================================================
    rviz_config = PathJoinSubstitution([bringup_share, 'rviz', 'clearrun.rviz'])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )
    
    # =========================================================================
    # System Monitor (optional diagnostic node)
    # =========================================================================
    # Can add a system monitor node here for health checks
    
    return LaunchDescription([
        # Arguments
        use_sim_time,
        launch_rviz,
        uav_fcu_url,
        ugv_fcu_url,
        map_file,
        
        # Log startup
        LogInfo(msg='==========================================='),
        LogInfo(msg='       CLEAR-RUN SYSTEM STARTUP           '),
        LogInfo(msg='  Autonomous FOD Detection & Removal      '),
        LogInfo(msg='==========================================='),
        
        # Launch groups
        uav_group,
        ugv_group,
        rviz_node,
    ])
