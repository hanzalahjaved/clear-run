#!/usr/bin/env python3
"""
Clear-Run Navigation Client

Provides a high-level interface to Nav2 for the UGV.
Handles goal sending, path following, and recovery behaviors.

Authors: Muhammad Hanzalah Javed, Aneeq
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState

from typing import Optional, Callable, List
from dataclasses import dataclass
from enum import Enum
import threading
import time
import math


class NavState(Enum):
    """Navigation client states."""
    IDLE = auto()
    NAVIGATING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()


@dataclass
class NavigationResult:
    """Result of a navigation action."""
    success: bool
    state: NavState
    message: str
    final_pose: Optional[PoseStamped] = None
    time_elapsed: float = 0.0


class NavigationClient(Node):
    """
    High-level client for Nav2 navigation stack.
    
    Provides methods for:
    - Navigating to a single pose
    - Following a series of waypoints
    - Setting initial pose
    - Clearing costmaps
    - Monitoring navigation state
    """
    
    def __init__(self, namespace: str = '/ugv'):
        super().__init__('navigation_client')
        
        self.ns = namespace
        self.state = NavState.IDLE
        self._current_goal_handle = None
        self._navigation_start_time: Optional[float] = None
        
        # Callbacks
        self._feedback_callback: Optional[Callable] = None
        self._result_callback: Optional[Callable] = None
        
        # Current robot pose
        self.current_pose: Optional[PoseStamped] = None
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Callback group
        self.cb_group = ReentrantCallbackGroup()
        
        # =====================================================================
        # Action Clients
        # =====================================================================
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            f'{self.ns}/navigate_to_pose',
            callback_group=self.cb_group
        )
        
        self.nav_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            f'{self.ns}/navigate_through_poses',
            callback_group=self.cb_group
        )
        
        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            f'{self.ns}/follow_waypoints',
            callback_group=self.cb_group
        )
        
        # =====================================================================
        # Service Clients
        # =====================================================================
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            f'{self.ns}/global_costmap/clear_entirely_global_costmap',
            callback_group=self.cb_group
        )
        
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            f'{self.ns}/local_costmap/clear_entirely_local_costmap',
            callback_group=self.cb_group
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'{self.ns}/initialpose',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'{self.ns}/cmd_vel',
            10
        )
        
        # =====================================================================
        # Subscribers
        # =====================================================================
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.ns}/odom',
            self._odom_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.plan_sub = self.create_subscription(
            Path,
            f'{self.ns}/plan',
            self._plan_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.current_plan: Optional[Path] = None
        
        self.get_logger().info(f'Navigation Client initialized for {self.ns}')
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def _odom_callback(self, msg: Odometry):
        """Handle odometry updates."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose
    
    def _plan_callback(self, msg: Path):
        """Handle plan updates."""
        self.current_plan = msg
    
    # =========================================================================
    # Navigation Methods
    # =========================================================================
    
    def wait_for_nav2(self, timeout: float = 30.0) -> bool:
        """Wait for Nav2 action servers to become available."""
        self.get_logger().info('Waiting for Nav2...')
        
        start = time.time()
        while (time.time() - start) < timeout:
            if self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Nav2 is ready')
                return True
        
        self.get_logger().error('Timeout waiting for Nav2')
        return False
    
    def navigate_to_pose(self, pose: PoseStamped, 
                        feedback_callback: Callable = None,
                        result_callback: Callable = None) -> bool:
        """
        Navigate to a goal pose.
        
        Args:
            pose: Target pose in map frame
            feedback_callback: Called with navigation feedback
            result_callback: Called when navigation completes
            
        Returns:
            True if goal was accepted
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('NavigateToPose action server not available')
            return False
        
        with self._lock:
            if self.state == NavState.NAVIGATING:
                self.get_logger().warn('Already navigating - cancelling previous goal')
                self.cancel_goal()
        
        self._feedback_callback = feedback_callback
        self._result_callback = result_callback
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(
            f'Navigating to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        
        self._navigation_start_time = time.time()
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._internal_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def navigate_through_poses(self, poses: List[PoseStamped],
                              feedback_callback: Callable = None,
                              result_callback: Callable = None) -> bool:
        """Navigate through a series of poses."""
        if not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('NavigateThroughPoses action server not available')
            return False
        
        with self._lock:
            if self.state == NavState.NAVIGATING:
                self.cancel_goal()
        
        self._feedback_callback = feedback_callback
        self._result_callback = result_callback
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        
        self.get_logger().info(f'Navigating through {len(poses)} poses')
        
        self._navigation_start_time = time.time()
        
        send_goal_future = self.nav_through_poses_client.send_goal_async(
            goal_msg,
            feedback_callback=self._internal_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def follow_waypoints(self, waypoints: List[PoseStamped],
                        result_callback: Callable = None) -> bool:
        """Follow a series of waypoints."""
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('FollowWaypoints action server not available')
            return False
        
        with self._lock:
            if self.state == NavState.NAVIGATING:
                self.cancel_goal()
        
        self._result_callback = result_callback
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        
        self.get_logger().info(f'Following {len(waypoints)} waypoints')
        
        self._navigation_start_time = time.time()
        
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def cancel_goal(self) -> bool:
        """Cancel the current navigation goal."""
        if self._current_goal_handle is None:
            return False
        
        self.get_logger().info('Cancelling navigation goal')
        cancel_future = self._current_goal_handle.cancel_goal_async()
        
        with self._lock:
            self.state = NavState.CANCELLED
        
        return True
    
    def stop(self):
        """Stop the robot immediately."""
        self.cancel_goal()
        
        # Send zero velocity
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        self.get_logger().info('Robot stopped')
    
    # =========================================================================
    # Action Callbacks
    # =========================================================================
    
    def _goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            with self._lock:
                self.state = NavState.FAILED
            
            if self._result_callback:
                result = NavigationResult(
                    success=False,
                    state=self.state,
                    message='Goal rejected'
                )
                self._result_callback(result)
            return
        
        self.get_logger().info('Navigation goal accepted')
        self._current_goal_handle = goal_handle
        
        with self._lock:
            self.state = NavState.NAVIGATING
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_response_callback)
    
    def _internal_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        
        if self._feedback_callback:
            self._feedback_callback(feedback)
    
    def _result_response_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        elapsed = time.time() - self._navigation_start_time if self._navigation_start_time else 0.0
        
        with self._lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.state = NavState.SUCCEEDED
                self.get_logger().info(f'Navigation succeeded in {elapsed:.1f}s')
            elif status == GoalStatus.STATUS_CANCELED:
                self.state = NavState.CANCELLED
                self.get_logger().info('Navigation cancelled')
            else:
                self.state = NavState.FAILED
                self.get_logger().error(f'Navigation failed with status: {status}')
        
        self._current_goal_handle = None
        
        if self._result_callback:
            nav_result = NavigationResult(
                success=(status == GoalStatus.STATUS_SUCCEEDED),
                state=self.state,
                message=f'Status: {status}',
                final_pose=self.current_pose,
                time_elapsed=elapsed
            )
            self._result_callback(nav_result)
    
    # =========================================================================
    # Utility Methods
    # =========================================================================
    
    def set_initial_pose(self, pose: PoseStamped, covariance: List[float] = None):
        """Set the robot's initial pose for localization."""
        msg = PoseWithCovarianceStamped()
        msg.header = pose.header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = pose.pose
        
        if covariance is None:
            # Default covariance (small uncertainty)
            covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        
        msg.pose.covariance = covariance
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('Initial pose set')
    
    def clear_costmaps(self) -> bool:
        """Clear both global and local costmaps."""
        success = True
        
        if self.clear_global_costmap_client.wait_for_service(timeout_sec=1.0):
            request = ClearEntireCostmap.Request()
            future = self.clear_global_costmap_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                success = False
        else:
            success = False
        
        if self.clear_local_costmap_client.wait_for_service(timeout_sec=1.0):
            request = ClearEntireCostmap.Request()
            future = self.clear_local_costmap_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                success = False
        else:
            success = False
        
        if success:
            self.get_logger().info('Costmaps cleared')
        else:
            self.get_logger().warn('Failed to clear some costmaps')
        
        return success
    
    def get_state(self) -> NavState:
        """Get current navigation state."""
        with self._lock:
            return self.state
    
    def is_navigating(self) -> bool:
        """Check if currently navigating."""
        with self._lock:
            return self.state == NavState.NAVIGATING
    
    def get_current_pose(self) -> Optional[PoseStamped]:
        """Get current robot pose."""
        return self.current_pose
    
    def get_distance_to_goal(self, goal: PoseStamped) -> float:
        """Calculate distance to a goal pose."""
        if self.current_pose is None:
            return float('inf')
        
        dx = goal.pose.position.x - self.current_pose.pose.position.x
        dy = goal.pose.position.y - self.current_pose.pose.position.y
        
        return math.sqrt(dx*dx + dy*dy)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()
    
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
