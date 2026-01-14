#!/usr/bin/env python3
"""
Clear-Run Simulation Launch

Launches the complete Clear-Run system in Gazebo simulation with ArduPilot SITL.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
    TimerAction,
    LogInfo,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =========================================================================
    # Package Shares
    # =========================================================================
    bringup_share = FindPackageShare('clearrun_bringup')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    world = DeclareLaunchArgument(
        'world',
        default_value='runway',
        description='Gazebo world to load'
    )
    
    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless'
    )
    
    launch_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    # =========================================================================
    # Gazebo Harmonic Simulation
    # =========================================================================
    # Note: Using Gazebo Harmonic (gz sim), not Gazebo Classic
    gazebo_world_path = '/home/gondal/Clear-Run/simulation/worlds/runway.sdf'
    
    gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', gazebo_world_path],
        output='screen'
    )
    
    # =========================================================================
    # ArduPilot SITL - UAV (ArduCopter)
    # Using native SITL mode (not gazebo-iris which requires plugin)
    # =========================================================================
    sitl_uav = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'ArduCopter',
            '-I', '0',
            '--no-mavproxy',
        ],
        cwd='/home/gondal/ardupilot/ArduCopter',
        output='screen',
    )
    
    # =========================================================================
    # ArduPilot SITL - UGV (ArduRover)
    # =========================================================================
    sitl_ugv = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'Rover',
            '-I', '1',
            '--no-mavproxy',
        ],
        cwd='/home/gondal/ardupilot/Rover',
        output='screen',
    )
    
    # =========================================================================
    # Main System Launch (delayed for SITL startup)
    # =========================================================================
    main_system = TimerAction(
        period=15.0,  # Wait for SITL to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        bringup_share,
                        'launch',
                        'clearrun_full.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'rviz': LaunchConfiguration('rviz'),
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        world,
        headless,
        launch_rviz,
        
        # Log startup
        LogInfo(msg='==========================================='),
        LogInfo(msg='     CLEAR-RUN SIMULATION STARTUP         '),
        LogInfo(msg='  Gazebo + ArduPilot SITL Environment     '),
        LogInfo(msg='==========================================='),
        
        # Start simulation components
        gazebo_cmd,
        sitl_uav,
        sitl_ugv,
        main_system,
    ])
