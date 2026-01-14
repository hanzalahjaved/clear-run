#!/usr/bin/env python3
"""
Clear-Run Visual Servoing Node

This node implements the "Stop and Center" visual servoing strategy.
When FOD is detected, the UAV switches to GUIDED mode and uses PID control
to center the object in the camera frame (nadir view), then logs the GPS coordinates.

This approach eliminates the need for complex 3D geometric calculations
and provides more accurate GPS localization of debris.

Authors: Muhammad Hanzalah Javed, Aneeq
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

from clearrun_msgs.msg import FodLocation

import numpy as np
from enum import Enum, auto
from typing import Optional
from dataclasses import dataclass
import time


class ServoState(Enum):
    """Visual servoing state machine states."""
    IDLE = auto()           # Waiting for detection
    CENTERING = auto()      # Actively centering on FOD
    HOLDING = auto()        # Centered, waiting for GPS stabilization
    LOGGING = auto()        # Recording GPS coordinates
    DISPATCHING = auto()    # Sending location to UGV
    RESUMING = auto()       # Returning to sweep pattern


@dataclass
class PIDController:
    """Simple PID controller for visual servoing."""
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    prev_error: float = 0.0
    integral_max: float = 100.0
    
    def compute(self, error: float, dt: float) -> float:
        """Compute PID output."""
        # Proportional
        p = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        i = self.ki * self.integral
        
        # Derivative
        d = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        return p + i + d
    
    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.prev_error = 0.0


class VisualServoNode(Node):
    """
    ROS 2 node for visual servoing to center FOD in camera frame.
    
    Workflow:
    1. Receive FOD detection with pixel coordinates
    2. Switch UAV to GUIDED mode
    3. Use PID to send velocity commands to center object
    4. Once centered, hold position and log GPS
    5. Publish accurate GPS to UGV
    6. Resume survey pattern
    """
    
    def __init__(self):
        super().__init__('visual_servo_node')
        
        # =====================================================================
        # Parameters
        # =====================================================================
        # Image frame parameters
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('centering_threshold', 20)  # pixels from center
        
        # PID gains for X (lateral) control
        self.declare_parameter('pid_x_kp', 0.5)
        self.declare_parameter('pid_x_ki', 0.01)
        self.declare_parameter('pid_x_kd', 0.1)
        
        # PID gains for Y (longitudinal) control
        self.declare_parameter('pid_y_kp', 0.5)
        self.declare_parameter('pid_y_ki', 0.01)
        self.declare_parameter('pid_y_kd', 0.1)
        
        # Velocity limits (m/s)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('min_velocity', 0.05)
        
        # Timing parameters
        self.declare_parameter('hold_duration', 2.0)  # seconds to hold for GPS averaging
        self.declare_parameter('gps_average_samples', 10)
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('detection_timeout', 3.0)  # seconds without detection
        
        # Altitude parameters
        self.declare_parameter('servo_altitude', 15.0)  # meters AGL for servoing
        
        # Get parameters
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.center_threshold = self.get_parameter('centering_threshold').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.min_vel = self.get_parameter('min_velocity').value
        self.hold_duration = self.get_parameter('hold_duration').value
        self.gps_samples = self.get_parameter('gps_average_samples').value
        self.control_rate = self.get_parameter('control_rate').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.servo_altitude = self.get_parameter('servo_altitude').value
        
        # Image center (target position for FOD)
        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2
        
        # =====================================================================
        # PID Controllers
        # =====================================================================
        self.pid_x = PIDController(
            kp=self.get_parameter('pid_x_kp').value,
            ki=self.get_parameter('pid_x_ki').value,
            kd=self.get_parameter('pid_x_kd').value
        )
        self.pid_y = PIDController(
            kp=self.get_parameter('pid_y_kp').value,
            ki=self.get_parameter('pid_y_ki').value,
            kd=self.get_parameter('pid_y_kd').value
        )
        
        # =====================================================================
        # State Variables
        # =====================================================================
        self.state = ServoState.IDLE
        self.current_detection: Optional[FodLocation] = None
        self.last_detection_time: Optional[float] = None
        self.hold_start_time: Optional[float] = None
        self.gps_buffer = []
        
        # MAVROS state
        self.mavros_state: Optional[State] = None
        self.current_gps: Optional[NavSatFix] = None
        self.is_armed = False
        self.current_mode = ""
        
        # Previous mode to return to after servoing
        self.previous_mode = "AUTO"
        
        # =====================================================================
        # Subscribers
        # =====================================================================
        self.detection_sub = self.create_subscription(
            FodLocation,
            '/uav/detection/fod',
            self.detection_callback,
            10
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/uav/mavros/state',
            self.state_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/uav/mavros/global_position/global',
            self.gps_callback,
            10
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        # Velocity commands for position control
        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/uav/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        # Final FOD location (with GPS) for UGV
        self.fod_location_pub = self.create_publisher(
            FodLocation,
            '/ugv/fod_target',
            10
        )
        
        # Status publisher
        self.servo_active_pub = self.create_publisher(
            Bool,
            '/uav/visual_servo/active',
            10
        )
        
        # =====================================================================
        # Service Clients
        # =====================================================================
        self.set_mode_client = self.create_client(SetMode, '/uav/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/uav/mavros/cmd/arming')
        
        # Wait for services
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        
        # =====================================================================
        # Control Loop Timer
        # =====================================================================
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        self.get_logger().info('Visual Servo Node initialized')
        self.get_logger().info(f'  Image size: {self.img_width}x{self.img_height}')
        self.get_logger().info(f'  Center threshold: {self.center_threshold} px')
        self.get_logger().info(f'  Control rate: {self.control_rate} Hz')
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def detection_callback(self, msg: FodLocation):
        """Handle incoming FOD detection."""
        self.current_detection = msg
        self.last_detection_time = time.time()
        
        # If we're idle, start servoing
        if self.state == ServoState.IDLE:
            self.get_logger().info(
                f'FOD detected: {msg.object_class} - Starting visual servo'
            )
            self._start_servoing()
    
    def state_callback(self, msg: State):
        """Handle MAVROS state updates."""
        self.mavros_state = msg
        self.is_armed = msg.armed
        self.current_mode = msg.mode
    
    def gps_callback(self, msg: NavSatFix):
        """Handle GPS updates."""
        self.current_gps = msg
        
        # If we're holding, collect GPS samples for averaging
        if self.state == ServoState.HOLDING:
            self.gps_buffer.append((msg.latitude, msg.longitude, msg.altitude))
    
    # =========================================================================
    # Control Loop
    # =========================================================================
    
    def control_loop(self):
        """Main control loop for visual servoing state machine."""
        # Publish servo status
        status_msg = Bool()
        status_msg.data = self.state != ServoState.IDLE
        self.servo_active_pub.publish(status_msg)
        
        # State machine
        if self.state == ServoState.IDLE:
            self._state_idle()
        elif self.state == ServoState.CENTERING:
            self._state_centering()
        elif self.state == ServoState.HOLDING:
            self._state_holding()
        elif self.state == ServoState.LOGGING:
            self._state_logging()
        elif self.state == ServoState.DISPATCHING:
            self._state_dispatching()
        elif self.state == ServoState.RESUMING:
            self._state_resuming()
    
    def _state_idle(self):
        """Idle state - waiting for detection."""
        # Detection callback handles transition to CENTERING
        pass
    
    def _state_centering(self):
        """Centering state - actively moving to center FOD."""
        # Check for detection timeout
        if self._check_detection_timeout():
            self.get_logger().warn('Detection lost - aborting servo')
            self._abort_servo()
            return
        
        if self.current_detection is None:
            return
        
        # Calculate pixel error from image center
        det = self.current_detection
        bbox_center_x = det.bbox_x + det.bbox_width // 2
        bbox_center_y = det.bbox_y + det.bbox_height // 2
        
        error_x = bbox_center_x - self.img_center_x  # Positive = object is right of center
        error_y = bbox_center_y - self.img_center_y  # Positive = object is below center
        
        # Check if centered
        if abs(error_x) < self.center_threshold and abs(error_y) < self.center_threshold:
            self.get_logger().info('FOD centered - transitioning to HOLDING')
            self._send_velocity(0.0, 0.0)  # Stop
            self.state = ServoState.HOLDING
            self.hold_start_time = time.time()
            self.gps_buffer = []
            self.pid_x.reset()
            self.pid_y.reset()
            return
        
        # Compute PID control
        dt = 1.0 / self.control_rate
        
        # Note: Camera frame to body frame mapping
        # Assuming nadir camera with X-right, Y-down in image
        # Maps to: body X-forward, Y-right
        vel_y = -self.pid_x.compute(error_x, dt)  # Lateral velocity (body Y)
        vel_x = -self.pid_y.compute(error_y, dt)  # Forward velocity (body X)
        
        # Normalize and scale to pixel error (larger error = faster movement)
        vel_x = np.clip(vel_x, -self.max_vel, self.max_vel)
        vel_y = np.clip(vel_y, -self.max_vel, self.max_vel)
        
        # Apply minimum velocity threshold to overcome static friction
        if abs(vel_x) < self.min_vel and abs(error_y) > self.center_threshold:
            vel_x = np.sign(vel_x) * self.min_vel
        if abs(vel_y) < self.min_vel and abs(error_x) > self.center_threshold:
            vel_y = np.sign(vel_y) * self.min_vel
        
        self._send_velocity(vel_x, vel_y)
        
        self.get_logger().debug(
            f'Centering: error=({error_x}, {error_y}) vel=({vel_x:.2f}, {vel_y:.2f})'
        )
    
    def _state_holding(self):
        """Holding state - maintaining position for GPS averaging."""
        # Keep sending zero velocity to hold position
        self._send_velocity(0.0, 0.0)
        
        # Check if we have enough GPS samples
        elapsed = time.time() - self.hold_start_time
        
        if elapsed >= self.hold_duration and len(self.gps_buffer) >= self.gps_samples:
            self.get_logger().info(
                f'GPS averaging complete ({len(self.gps_buffer)} samples) - logging'
            )
            self.state = ServoState.LOGGING
    
    def _state_logging(self):
        """Logging state - compute averaged GPS and prepare message."""
        if not self.gps_buffer:
            self.get_logger().error('No GPS samples - aborting')
            self._abort_servo()
            return
        
        # Compute average GPS
        lats = [g[0] for g in self.gps_buffer]
        lons = [g[1] for g in self.gps_buffer]
        alts = [g[2] for g in self.gps_buffer]
        
        avg_lat = np.mean(lats)
        avg_lon = np.mean(lons)
        avg_alt = np.mean(alts)
        
        # Standard deviations for quality check
        std_lat = np.std(lats)
        std_lon = np.std(lons)
        
        self.get_logger().info(
            f'Logged GPS: ({avg_lat:.7f}, {avg_lon:.7f}) ± ({std_lat:.7f}, {std_lon:.7f})'
        )
        
        # Update detection with GPS
        if self.current_detection is not None:
            self.current_detection.latitude = avg_lat
            self.current_detection.longitude = avg_lon
            self.current_detection.altitude = avg_alt
            self.current_detection.is_centered = True
        
        self.state = ServoState.DISPATCHING
    
    def _state_dispatching(self):
        """Dispatching state - send FOD location to UGV."""
        if self.current_detection is None:
            self._abort_servo()
            return
        
        # Create message for UGV
        fod_msg = self.current_detection
        fod_msg.header.stamp = self.get_clock().now().to_msg()
        fod_msg.header.frame_id = 'map'
        
        # Publish to UGV
        self.fod_location_pub.publish(fod_msg)
        
        self.get_logger().info(
            f'Dispatched FOD #{fod_msg.fod_id} to UGV: '
            f'{fod_msg.object_class} at ({fod_msg.latitude:.7f}, {fod_msg.longitude:.7f})'
        )
        
        self.state = ServoState.RESUMING
    
    def _state_resuming(self):
        """Resuming state - return to survey pattern."""
        # Switch back to previous mode (AUTO for waypoint mission)
        self._set_mode(self.previous_mode)
        
        self.get_logger().info(f'Resuming {self.previous_mode} mode')
        
        # Reset state
        self._reset_servo()
    
    # =========================================================================
    # Helper Methods
    # =========================================================================
    
    def _start_servoing(self):
        """Initialize visual servoing sequence."""
        # Save current mode to return to later
        if self.mavros_state is not None:
            self.previous_mode = self.mavros_state.mode
        
        # Switch to GUIDED mode
        if not self._set_mode('GUIDED'):
            self.get_logger().error('Failed to switch to GUIDED mode')
            return
        
        self.state = ServoState.CENTERING
        self.pid_x.reset()
        self.pid_y.reset()
    
    def _abort_servo(self):
        """Abort servoing and return to previous mode."""
        self.get_logger().warn('Aborting visual servo')
        self._set_mode(self.previous_mode)
        self._reset_servo()
    
    def _reset_servo(self):
        """Reset servo state to idle."""
        self.state = ServoState.IDLE
        self.current_detection = None
        self.last_detection_time = None
        self.hold_start_time = None
        self.gps_buffer = []
        self.pid_x.reset()
        self.pid_y.reset()
    
    def _check_detection_timeout(self) -> bool:
        """Check if detection has timed out."""
        if self.last_detection_time is None:
            return True
        return (time.time() - self.last_detection_time) > self.detection_timeout
    
    def _send_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """Send velocity command to MAVROS."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = vx  # Forward
        msg.twist.linear.y = vy  # Left
        msg.twist.linear.z = vz  # Up
        
        # No angular velocity (maintain heading)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.vel_pub.publish(msg)
    
    def _set_mode(self, mode: str) -> bool:
        """Set MAVROS flight mode."""
        if not self.set_mode_client.service_is_ready():
            self.get_logger().error('set_mode service not available')
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
                return True
            else:
                self.get_logger().error(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error(f'Set mode service call failed')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
