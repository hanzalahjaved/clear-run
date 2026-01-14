#!/usr/bin/env python3
"""
Clear-Run FOD Retriever Node

Main coordinator for the UGV. Receives FOD locations from the UAV,
plans paths using Nav2, navigates to the debris, and commands the
scoop mechanism to collect it.

Authors: Muhammad Hanzalah Javed, Aneeq
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from clearrun_msgs.msg import FodLocation, ScoopCommand, ScoopStatus

import numpy as np
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, List
from queue import PriorityQueue
import threading
import time
import math


class RetrieverState(Enum):
    """UGV retriever state machine states."""
    IDLE = auto()           # Waiting for FOD targets
    PLANNING = auto()       # Converting GPS to local coordinates
    NAVIGATING = auto()     # Nav2 in progress
    APPROACHING = auto()    # Final approach to FOD
    COLLECTING = auto()     # Scoop operation in progress
    VERIFYING = auto()      # Checking collection success
    RETURNING = auto()      # Returning to base (optional)
    ERROR = auto()          # Error state


@dataclass(order=True)
class FodTarget:
    """Priority queue item for FOD targets."""
    priority: float  # Distance or urgency
    fod: FodLocation = field(compare=False)


class FodRetrieverNode(Node):
    """
    ROS 2 node for autonomous FOD collection by the UGV.
    
    Receives FOD locations from the UAV, manages a priority queue,
    navigates using Nav2, and controls the scoop mechanism.
    """
    
    def __init__(self):
        super().__init__('fod_retriever_node')
        
        # =====================================================================
        # Parameters
        # =====================================================================
        self.declare_parameter('home_latitude', 0.0)
        self.declare_parameter('home_longitude', 0.0)
        self.declare_parameter('approach_distance', 0.5)  # meters from target
        self.declare_parameter('collection_timeout', 30.0)  # seconds
        self.declare_parameter('max_queue_size', 50)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('return_to_home', False)
        
        self.home_lat = self.get_parameter('home_latitude').value
        self.home_lon = self.get_parameter('home_longitude').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.collection_timeout = self.get_parameter('collection_timeout').value
        self.max_queue = self.get_parameter('max_queue_size').value
        self.auto_start = self.get_parameter('auto_start').value
        self.return_home = self.get_parameter('return_to_home').value
        
        # =====================================================================
        # State Variables
        # =====================================================================
        self.state = RetrieverState.IDLE
        self.fod_queue: PriorityQueue = PriorityQueue()
        self.current_target: Optional[FodLocation] = None
        self.collected_count = 0
        self.failed_count = 0
        
        # Current pose
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_pose: Optional[PoseStamped] = None
        
        # Scoop status
        self.scoop_status: Optional[ScoopStatus] = None
        
        # Thread safety
        self.state_lock = threading.Lock()
        
        # Callback group for concurrent callbacks
        self.cb_group = ReentrantCallbackGroup()
        
        # =====================================================================
        # Subscribers
        # =====================================================================
        self.fod_target_sub = self.create_subscription(
            FodLocation,
            '/ugv/fod_target',
            self.fod_target_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/ugv/mavros/global_position/global',
            self.gps_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ugv/odom',
            self.odom_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.scoop_status_sub = self.create_subscription(
            ScoopStatus,
            '/ugv/scoop/status',
            self.scoop_status_callback,
            10,
            callback_group=self.cb_group
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.scoop_cmd_pub = self.create_publisher(
            ScoopCommand,
            '/ugv/scoop/command',
            10
        )
        
        self.state_pub = self.create_publisher(
            String,
            '/ugv/retriever/state',
            10
        )
        
        self.collected_pub = self.create_publisher(
            FodLocation,
            '/ugv/retriever/collected',
            10
        )
        
        # =====================================================================
        # Action Clients
        # =====================================================================
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/ugv/navigate_to_pose'
        )
        
        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server available')
        
        # =====================================================================
        # Main Loop Timer
        # =====================================================================
        self.main_timer = self.create_timer(0.5, self.main_loop)
        
        self.get_logger().info('FOD Retriever Node initialized')
        self.get_logger().info(f'  Approach distance: {self.approach_distance}m')
        self.get_logger().info(f'  Auto start: {self.auto_start}')
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def fod_target_callback(self, msg: FodLocation):
        """Handle incoming FOD target from UAV."""
        if self.fod_queue.qsize() >= self.max_queue:
            self.get_logger().warn('FOD queue full - dropping target')
            return
        
        # Calculate priority based on distance (closer = higher priority)
        distance = self._calculate_distance(
            self.current_lat, self.current_lon,
            msg.latitude, msg.longitude
        )
        
        # Create priority queue item (lower value = higher priority)
        target = FodTarget(priority=distance, fod=msg)
        self.fod_queue.put(target)
        
        self.get_logger().info(
            f'FOD target received: #{msg.fod_id} ({msg.object_class}) '
            f'at distance {distance:.1f}m - Queue size: {self.fod_queue.qsize()}'
        )
    
    def gps_callback(self, msg: NavSatFix):
        """Handle GPS updates."""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
    
    def odom_callback(self, msg: Odometry):
        """Handle odometry updates."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose
    
    def scoop_status_callback(self, msg: ScoopStatus):
        """Handle scoop mechanism status."""
        self.scoop_status = msg
    
    # =========================================================================
    # Main Loop
    # =========================================================================
    
    def main_loop(self):
        """Main state machine loop."""
        # Publish current state
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        
        with self.state_lock:
            if self.state == RetrieverState.IDLE:
                self._state_idle()
            elif self.state == RetrieverState.PLANNING:
                self._state_planning()
            elif self.state == RetrieverState.NAVIGATING:
                pass  # Handled by action callback
            elif self.state == RetrieverState.APPROACHING:
                self._state_approaching()
            elif self.state == RetrieverState.COLLECTING:
                self._state_collecting()
            elif self.state == RetrieverState.VERIFYING:
                self._state_verifying()
            elif self.state == RetrieverState.RETURNING:
                self._state_returning()
            elif self.state == RetrieverState.ERROR:
                self._state_error()
    
    def _state_idle(self):
        """Idle state - check for new targets."""
        if not self.fod_queue.empty() and self.auto_start:
            target = self.fod_queue.get()
            self.current_target = target.fod
            self.state = RetrieverState.PLANNING
            self.get_logger().info(
                f'Starting retrieval of FOD #{self.current_target.fod_id}'
            )
    
    def _state_planning(self):
        """Planning state - convert GPS to local coordinates and plan path."""
        if self.current_target is None:
            self.state = RetrieverState.IDLE
            return
        
        # Convert GPS to local map frame coordinates
        # This uses a simple local tangent plane approximation
        target_pose = self._gps_to_local(
            self.current_target.latitude,
            self.current_target.longitude
        )
        
        if target_pose is None:
            self.get_logger().error('Failed to convert GPS to local coordinates')
            self.state = RetrieverState.ERROR
            return
        
        # Send navigation goal
        self._send_nav_goal(target_pose)
        self.state = RetrieverState.NAVIGATING
    
    def _state_approaching(self):
        """Final approach state - slow approach to FOD."""
        # The navigation action brings us close, this handles final positioning
        # For now, we proceed directly to collection
        self.state = RetrieverState.COLLECTING
        self._start_collection()
    
    def _state_collecting(self):
        """Collection state - operating the scoop."""
        if self.scoop_status is None:
            return
        
        # Check if collection is complete
        if self.scoop_status.current_state == ScoopStatus.STATE_RETRACTED:
            self.get_logger().info('Scoop retracted - verifying collection')
            self.state = RetrieverState.VERIFYING
        elif self.scoop_status.current_state == ScoopStatus.STATE_ERROR:
            self.get_logger().error('Scoop error during collection')
            self.failed_count += 1
            self.state = RetrieverState.ERROR
    
    def _state_verifying(self):
        """Verification state - check if FOD was collected."""
        # In a real system, this would use sensors in the scoop
        # For now, we assume success if no error
        if self.scoop_status and self.scoop_status.object_detected:
            self.collected_count += 1
            
            # Mark as collected and publish
            if self.current_target:
                self.current_target.is_collected = True
                self.collected_pub.publish(self.current_target)
                self.get_logger().info(
                    f'FOD #{self.current_target.fod_id} collected successfully! '
                    f'Total: {self.collected_count}'
                )
        else:
            self.get_logger().warn('Collection verification failed')
            if self.current_target:
                self.current_target.requires_manual = True
                self.failed_count += 1
        
        # Move to next target or return home
        self.current_target = None
        if self.return_home and self.fod_queue.empty():
            self.state = RetrieverState.RETURNING
        else:
            self.state = RetrieverState.IDLE
    
    def _state_returning(self):
        """Return to home state."""
        home_pose = self._gps_to_local(self.home_lat, self.home_lon)
        if home_pose:
            self._send_nav_goal(home_pose)
        self.state = RetrieverState.IDLE
    
    def _state_error(self):
        """Error state - attempt recovery or wait for operator."""
        # Retract scoop if deployed
        self._send_scoop_command(ScoopCommand.CMD_RETRACT)
        
        # Could implement retry logic here
        self.current_target = None
        self.state = RetrieverState.IDLE
    
    # =========================================================================
    # Navigation
    # =========================================================================
    
    def _send_nav_goal(self, pose: PoseStamped):
        """Send navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(
            f'Navigating to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_goal_response_callback)
    
    def _nav_goal_response_callback(self, future):
        """Handle Nav2 goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.state = RetrieverState.ERROR
            return
        
        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)
    
    def _nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback."""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {distance:.2f}m')
    
    def _nav_result_callback(self, future):
        """Handle Nav2 result."""
        result = future.result()
        status = result.status
        
        with self.state_lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded')
                self.state = RetrieverState.APPROACHING
            else:
                self.get_logger().error(f'Navigation failed with status: {status}')
                self.state = RetrieverState.ERROR
    
    # =========================================================================
    # Scoop Control
    # =========================================================================
    
    def _start_collection(self):
        """Start the collection sequence."""
        self.get_logger().info('Starting collection sequence')
        self._send_scoop_command(ScoopCommand.CMD_COLLECT)
    
    def _send_scoop_command(self, command: int, brush_speed: float = 1.0):
        """Send command to scoop mechanism."""
        msg = ScoopCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = command
        msg.brush_speed = brush_speed
        msg.scoop_angle = 0.0
        
        self.scoop_cmd_pub.publish(msg)
    
    # =========================================================================
    # Coordinate Transforms
    # =========================================================================
    
    def _gps_to_local(self, lat: float, lon: float) -> Optional[PoseStamped]:
        """
        Convert GPS coordinates to local map frame.
        Uses a simple local tangent plane (LTP) approximation.
        """
        if self.home_lat == 0.0 and self.home_lon == 0.0:
            self.get_logger().warn('Home position not set - using current as reference')
            self.home_lat = self.current_lat
            self.home_lon = self.current_lon
        
        # Earth radius in meters
        R = 6371000.0
        
        # Convert to radians
        lat1 = math.radians(self.home_lat)
        lat2 = math.radians(lat)
        dlon = math.radians(lon - self.home_lon)
        dlat = math.radians(lat - self.home_lat)
        
        # Local tangent plane approximation
        x = R * dlon * math.cos((lat1 + lat2) / 2)
        y = R * dlat
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        
        return pose
    
    def _calculate_distance(self, lat1: float, lon1: float, 
                           lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points using Haversine formula."""
        R = 6371000.0  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c


def main(args=None):
    rclpy.init(args=args)
    node = FodRetrieverNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
