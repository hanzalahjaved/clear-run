#!/usr/bin/env python3
"""
Clear-Run MAVROS Interface

Provides a high-level interface to ArduPilot through MAVROS.
Handles arming, mode changes, takeoff, landing, and telemetry.

Authors: Muhammad Hanzalah Javed, Aneeq
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, BatteryState, Imu
from mavros_msgs.msg import State, HomePosition, WaypointList
from mavros_msgs.srv import (
    CommandBool, SetMode, CommandTOL, CommandLong, 
    WaypointPush, WaypointClear, WaypointSetCurrent
)
from std_msgs.msg import Bool, Float64

from typing import Optional, Callable, List
from dataclasses import dataclass
from enum import Enum
import threading
import time


class FlightMode(Enum):
    """ArduCopter flight modes."""
    STABILIZE = "STABILIZE"
    ALT_HOLD = "ALT_HOLD"
    LOITER = "LOITER"
    RTL = "RTL"
    AUTO = "AUTO"
    GUIDED = "GUIDED"
    LAND = "LAND"
    BRAKE = "BRAKE"


@dataclass
class Telemetry:
    """Current UAV telemetry data."""
    # State
    armed: bool = False
    mode: str = ""
    connected: bool = False
    
    # Position (GPS)
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_msl: float = 0.0
    altitude_rel: float = 0.0
    
    # Attitude (degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    
    # Battery
    voltage: float = 0.0
    current: float = 0.0
    percentage: float = 0.0
    
    # Home position
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0


class MAVROSInterface(Node):
    """
    High-level interface to ArduPilot through MAVROS.
    
    Provides methods for:
    - Arming/Disarming
    - Mode changes
    - Takeoff/Landing
    - Waypoint missions
    - Velocity commands
    - Telemetry access
    """
    
    def __init__(self, namespace: str = '/uav'):
        super().__init__('mavros_interface')
        
        self.ns = namespace
        self.telemetry = Telemetry()
        
        # Callback groups for concurrent service calls
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        
        # =====================================================================
        # Subscribers
        # =====================================================================
        self.state_sub = self.create_subscription(
            State,
            f'{self.ns}/state',
            self._state_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'{self.ns}/global_position/global',
            self._gps_callback,
            10
        )
        
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            f'{self.ns}/local_position/pose',
            self._local_pose_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            f'{self.ns}/local_position/velocity_local',
            self._velocity_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            f'{self.ns}/imu/data',
            self._imu_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'{self.ns}/battery',
            self._battery_callback,
            10
        )
        
        self.home_sub = self.create_subscription(
            HomePosition,
            f'{self.ns}/home_position/home',
            self._home_callback,
            10
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.setpoint_position_pub = self.create_publisher(
            PoseStamped,
            f'{self.ns}/setpoint_position/local',
            10
        )
        
        self.setpoint_velocity_pub = self.create_publisher(
            TwistStamped,
            f'{self.ns}/setpoint_velocity/cmd_vel',
            10
        )
        
        # =====================================================================
        # Service Clients
        # =====================================================================
        self.arm_client = self.create_client(
            CommandBool, 
            f'{self.ns}/cmd/arming',
            callback_group=self.service_cb_group
        )
        
        self.set_mode_client = self.create_client(
            SetMode, 
            f'{self.ns}/set_mode',
            callback_group=self.service_cb_group
        )
        
        self.takeoff_client = self.create_client(
            CommandTOL, 
            f'{self.ns}/cmd/takeoff',
            callback_group=self.service_cb_group
        )
        
        self.land_client = self.create_client(
            CommandTOL, 
            f'{self.ns}/cmd/land',
            callback_group=self.service_cb_group
        )
        
        self.command_client = self.create_client(
            CommandLong,
            f'{self.ns}/cmd/command',
            callback_group=self.service_cb_group
        )
        
        self.waypoint_push_client = self.create_client(
            WaypointPush,
            f'{self.ns}/mission/push',
            callback_group=self.service_cb_group
        )
        
        self.waypoint_clear_client = self.create_client(
            WaypointClear,
            f'{self.ns}/mission/clear',
            callback_group=self.service_cb_group
        )
        
        self.get_logger().info(f'MAVROS Interface initialized for {self.ns}')
    
    # =========================================================================
    # Subscriber Callbacks
    # =========================================================================
    
    def _state_callback(self, msg: State):
        """Handle state updates."""
        self.telemetry.armed = msg.armed
        self.telemetry.mode = msg.mode
        self.telemetry.connected = msg.connected
    
    def _gps_callback(self, msg: NavSatFix):
        """Handle GPS updates."""
        self.telemetry.latitude = msg.latitude
        self.telemetry.longitude = msg.longitude
        self.telemetry.altitude_msl = msg.altitude
    
    def _local_pose_callback(self, msg: PoseStamped):
        """Handle local pose updates."""
        self.telemetry.altitude_rel = msg.pose.position.z
    
    def _velocity_callback(self, msg: TwistStamped):
        """Handle velocity updates."""
        self.telemetry.vx = msg.twist.linear.x
        self.telemetry.vy = msg.twist.linear.y
        self.telemetry.vz = msg.twist.linear.z
    
    def _imu_callback(self, msg: Imu):
        """Handle IMU updates - extract attitude."""
        import math
        # Convert quaternion to euler angles
        q = msg.orientation
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.telemetry.roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.telemetry.pitch = math.degrees(math.copysign(math.pi / 2, sinp))
        else:
            self.telemetry.pitch = math.degrees(math.asin(sinp))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.telemetry.yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    
    def _battery_callback(self, msg: BatteryState):
        """Handle battery updates."""
        self.telemetry.voltage = msg.voltage
        self.telemetry.current = msg.current
        self.telemetry.percentage = msg.percentage
    
    def _home_callback(self, msg: HomePosition):
        """Handle home position updates."""
        self.telemetry.home_lat = msg.geo.latitude
        self.telemetry.home_lon = msg.geo.longitude
        self.telemetry.home_alt = msg.geo.altitude
    
    # =========================================================================
    # Command Methods
    # =========================================================================
    
    def wait_for_connection(self, timeout: float = 30.0) -> bool:
        """Wait for MAVROS connection to FCU."""
        start = time.time()
        while (time.time() - start) < timeout:
            if self.telemetry.connected:
                self.get_logger().info('Connected to FCU')
                return True
            time.sleep(0.1)
        self.get_logger().error('Timeout waiting for FCU connection')
        return False
    
    def arm(self, timeout: float = 10.0) -> bool:
        """Arm the vehicle."""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arm service not available')
            return False
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() and future.result().success:
            self.get_logger().info('Vehicle armed')
            return True
        else:
            self.get_logger().error('Failed to arm')
            return False
    
    def disarm(self, force: bool = False, timeout: float = 10.0) -> bool:
        """Disarm the vehicle."""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arm service not available')
            return False
        
        request = CommandBool.Request()
        request.value = False
        
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() and future.result().success:
            self.get_logger().info('Vehicle disarmed')
            return True
        else:
            self.get_logger().error('Failed to disarm')
            return False
    
    def set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        """Set flight mode."""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set mode service not available')
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Mode set to {mode}')
            return True
        else:
            self.get_logger().error(f'Failed to set mode to {mode}')
            return False
    
    def takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        """Command takeoff to specified altitude."""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Takeoff service not available')
            return False
        
        request = CommandTOL.Request()
        request.altitude = altitude
        request.latitude = float('nan')  # Use current position
        request.longitude = float('nan')
        request.min_pitch = 0.0
        request.yaw = float('nan')  # Maintain current yaw
        
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() and future.result().success:
            self.get_logger().info(f'Takeoff to {altitude}m commanded')
            return True
        else:
            self.get_logger().error('Takeoff command failed')
            return False
    
    def land(self, timeout: float = 30.0) -> bool:
        """Command landing."""
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Land service not available')
            return False
        
        request = CommandTOL.Request()
        request.altitude = 0.0
        request.latitude = float('nan')
        request.longitude = float('nan')
        request.min_pitch = 0.0
        request.yaw = float('nan')
        
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() and future.result().success:
            self.get_logger().info('Land commanded')
            return True
        else:
            self.get_logger().error('Land command failed')
            return False
    
    def rtl(self) -> bool:
        """Return to launch."""
        return self.set_mode('RTL')
    
    # =========================================================================
    # Setpoint Methods
    # =========================================================================
    
    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Send velocity setpoint (body frame)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        
        self.setpoint_velocity_pub.publish(msg)
    
    def set_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Send position setpoint (local frame)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Convert yaw to quaternion
        import math
        msg.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.orientation.z = math.sin(yaw / 2)
        
        self.setpoint_position_pub.publish(msg)
    
    # =========================================================================
    # Utility Methods
    # =========================================================================
    
    def get_telemetry(self) -> Telemetry:
        """Get current telemetry snapshot."""
        return self.telemetry
    
    def is_armed(self) -> bool:
        """Check if vehicle is armed."""
        return self.telemetry.armed
    
    def is_connected(self) -> bool:
        """Check if connected to FCU."""
        return self.telemetry.connected
    
    def get_mode(self) -> str:
        """Get current flight mode."""
        return self.telemetry.mode


def main(args=None):
    rclpy.init(args=args)
    node = MAVROSInterface()
    
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
