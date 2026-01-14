#!/usr/bin/env python3
"""
Clear-Run Scoop Controller

Controls the active brush-assisted scoop mechanism on the UGV.
Manages the scoop actuator and inward-rotating brushes for FOD collection.

The novelty of this design is the rotating brushes at the intake lip,
which provide mechanical grip to sweep flat objects (like washers or
luggage tags) that have high static friction.

Authors: Muhammad Hanzalah Javed, Aneeq
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from clearrun_msgs.msg import ScoopCommand, ScoopStatus

from enum import Enum, auto
from typing import Optional
import time


class ScoopState(Enum):
    """Scoop mechanism states."""
    IDLE = auto()
    DEPLOYING = auto()
    DEPLOYED = auto()
    COLLECTING = auto()
    RETRACTING = auto()
    RETRACTED = auto()
    ERROR = auto()


class ScoopController(Node):
    """
    ROS 2 node for controlling the active brush-assisted scoop.
    
    Hardware interface:
    - Scoop actuator: Linear actuator or servo for raising/lowering
    - Brush motors: DC motors with encoder feedback for inward rotation
    - Proximity sensor: Detects object presence in collection bin
    - Current sensor: Monitors brush motor load
    
    Collection sequence:
    1. Deploy scoop (lower to ground)
    2. Start brushes (inward rotation)
    3. Drive forward slowly (controlled by main node)
    4. Stop brushes
    5. Retract scoop
    """
    
    def __init__(self):
        super().__init__('scoop_controller')
        
        # =====================================================================
        # Parameters
        # =====================================================================
        self.declare_parameter('deploy_time', 2.0)       # seconds to fully deploy
        self.declare_parameter('retract_time', 2.0)      # seconds to fully retract
        self.declare_parameter('brush_startup_time', 0.5)
        self.declare_parameter('collection_duration', 5.0)
        self.declare_parameter('brush_speed_default', 0.8)
        self.declare_parameter('current_threshold', 5.0)  # amps - indicates stall
        
        self.deploy_time = self.get_parameter('deploy_time').value
        self.retract_time = self.get_parameter('retract_time').value
        self.brush_startup = self.get_parameter('brush_startup_time').value
        self.collection_duration = self.get_parameter('collection_duration').value
        self.default_brush_speed = self.get_parameter('brush_speed_default').value
        self.current_threshold = self.get_parameter('current_threshold').value
        
        # =====================================================================
        # State Variables
        # =====================================================================
        self.state = ScoopState.IDLE
        self.scoop_position = 0.0  # 0.0 = up, 1.0 = down
        self.brush_speed = 0.0
        self.brush_current = 0.0
        self.object_detected = False
        self.bin_full = False
        
        self.operation_start_time: Optional[float] = None
        self.target_brush_speed = 0.0
        
        # Error tracking
        self.error_message = ""
        self.brush_motor_ok = True
        self.scoop_actuator_ok = True
        
        # =====================================================================
        # Subscribers
        # =====================================================================
        self.command_sub = self.create_subscription(
            ScoopCommand,
            '/ugv/scoop/command',
            self.command_callback,
            10
        )
        
        # Simulated sensor inputs (replace with actual hardware interfaces)
        self.proximity_sub = self.create_subscription(
            Bool,
            '/ugv/scoop/proximity_sensor',
            self.proximity_callback,
            10
        )
        
        self.current_sub = self.create_subscription(
            Float32,
            '/ugv/scoop/motor_current',
            self.current_callback,
            10
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.status_pub = self.create_publisher(
            ScoopStatus,
            '/ugv/scoop/status',
            10
        )
        
        # Hardware output publishers (for simulation/actual hardware)
        self.actuator_pub = self.create_publisher(
            Float32,
            '/ugv/scoop/actuator_cmd',  # 0.0 = up, 1.0 = down
            10
        )
        
        self.brush_pub = self.create_publisher(
            Float32,
            '/ugv/scoop/brush_cmd',  # -1.0 to 1.0 (negative = inward)
            10
        )
        
        # =====================================================================
        # Control Loop Timer
        # =====================================================================
        self.control_rate = 20.0  # Hz
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        # Status publishing timer
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('Scoop Controller initialized')
        self.get_logger().info(f'  Deploy time: {self.deploy_time}s')
        self.get_logger().info(f'  Collection duration: {self.collection_duration}s')
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def command_callback(self, msg: ScoopCommand):
        """Handle scoop commands."""
        cmd = msg.command
        
        self.get_logger().info(f'Received scoop command: {cmd}')
        
        if cmd == ScoopCommand.CMD_IDLE:
            self._cmd_idle()
        elif cmd == ScoopCommand.CMD_DEPLOY:
            self._cmd_deploy()
        elif cmd == ScoopCommand.CMD_RETRACT:
            self._cmd_retract()
        elif cmd == ScoopCommand.CMD_BRUSH_START:
            self._cmd_brush_start(msg.brush_speed)
        elif cmd == ScoopCommand.CMD_BRUSH_STOP:
            self._cmd_brush_stop()
        elif cmd == ScoopCommand.CMD_COLLECT:
            self._cmd_collect(msg.brush_speed)
    
    def proximity_callback(self, msg: Bool):
        """Handle proximity sensor updates."""
        self.object_detected = msg.data
    
    def current_callback(self, msg: Float32):
        """Handle motor current updates."""
        self.brush_current = msg.data
        
        # Check for overcurrent (stall)
        if self.brush_current > self.current_threshold:
            self.get_logger().warn('Brush motor overcurrent detected')
            self._emergency_stop()
    
    # =========================================================================
    # Command Handlers
    # =========================================================================
    
    def _cmd_idle(self):
        """Stop all operations and return to idle."""
        self._stop_brushes()
        self.state = ScoopState.IDLE
    
    def _cmd_deploy(self):
        """Deploy (lower) the scoop."""
        if self.state not in [ScoopState.IDLE, ScoopState.RETRACTED]:
            self.get_logger().warn('Cannot deploy - not in idle state')
            return
        
        self.state = ScoopState.DEPLOYING
        self.operation_start_time = time.time()
        self.get_logger().info('Deploying scoop')
    
    def _cmd_retract(self):
        """Retract (raise) the scoop."""
        self._stop_brushes()
        self.state = ScoopState.RETRACTING
        self.operation_start_time = time.time()
        self.get_logger().info('Retracting scoop')
    
    def _cmd_brush_start(self, speed: float = None):
        """Start the brushes."""
        if speed is None or speed <= 0:
            speed = self.default_brush_speed
        
        self.target_brush_speed = speed
        self.get_logger().info(f'Starting brushes at {speed*100:.0f}% speed')
    
    def _cmd_brush_stop(self):
        """Stop the brushes."""
        self._stop_brushes()
        self.get_logger().info('Stopping brushes')
    
    def _cmd_collect(self, brush_speed: float = None):
        """
        Execute full collection sequence.
        1. Deploy scoop
        2. Start brushes
        3. Run for collection duration
        4. Stop brushes
        5. Retract scoop
        """
        if self.state not in [ScoopState.IDLE, ScoopState.RETRACTED]:
            self.get_logger().warn('Cannot start collection - not in idle state')
            return
        
        if brush_speed is None or brush_speed <= 0:
            brush_speed = self.default_brush_speed
        
        self.target_brush_speed = brush_speed
        self.state = ScoopState.DEPLOYING
        self.operation_start_time = time.time()
        self.get_logger().info('Starting collection sequence')
    
    def _stop_brushes(self):
        """Stop the brush motors."""
        self.target_brush_speed = 0.0
        self.brush_speed = 0.0
    
    def _emergency_stop(self):
        """Emergency stop all operations."""
        self._stop_brushes()
        self.state = ScoopState.ERROR
        self.error_message = "Emergency stop - overcurrent"
        self.brush_motor_ok = False
        self.get_logger().error('EMERGENCY STOP')
    
    # =========================================================================
    # Control Loop
    # =========================================================================
    
    def control_loop(self):
        """Main control loop - handles state transitions and actuator control."""
        current_time = time.time()
        
        if self.state == ScoopState.IDLE:
            # Hold position
            pass
            
        elif self.state == ScoopState.DEPLOYING:
            # Lower the scoop
            elapsed = current_time - self.operation_start_time
            self.scoop_position = min(1.0, elapsed / self.deploy_time)
            
            if self.scoop_position >= 1.0:
                self.state = ScoopState.DEPLOYED
                self.get_logger().info('Scoop deployed')
                
                # If this is part of collection sequence, start brushes
                if self.target_brush_speed > 0:
                    self.state = ScoopState.COLLECTING
                    self.operation_start_time = current_time
                    
        elif self.state == ScoopState.DEPLOYED:
            # Holding deployed position, waiting for command
            pass
            
        elif self.state == ScoopState.COLLECTING:
            # Brushes running, collecting debris
            self.brush_speed = self.target_brush_speed
            
            elapsed = current_time - self.operation_start_time
            if elapsed >= self.collection_duration:
                self.get_logger().info('Collection complete - retracting')
                self._stop_brushes()
                self.state = ScoopState.RETRACTING
                self.operation_start_time = current_time
                
        elif self.state == ScoopState.RETRACTING:
            # Raise the scoop
            elapsed = current_time - self.operation_start_time
            self.scoop_position = max(0.0, 1.0 - elapsed / self.retract_time)
            
            if self.scoop_position <= 0.0:
                self.state = ScoopState.RETRACTED
                self.get_logger().info('Scoop retracted')
                
        elif self.state == ScoopState.RETRACTED:
            # Holding retracted position
            pass
            
        elif self.state == ScoopState.ERROR:
            # Error state - hold position
            self.scoop_position = 0.0  # Retract for safety
            self.brush_speed = 0.0
        
        # Send actuator commands
        self._send_actuator_commands()
    
    def _send_actuator_commands(self):
        """Send commands to hardware actuators."""
        # Scoop actuator
        actuator_msg = Float32()
        actuator_msg.data = self.scoop_position
        self.actuator_pub.publish(actuator_msg)
        
        # Brush motors (negative = inward rotation)
        brush_msg = Float32()
        brush_msg.data = -self.brush_speed  # Inward rotation
        self.brush_pub.publish(brush_msg)
    
    def publish_status(self):
        """Publish current scoop status."""
        msg = ScoopStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Map internal state to message state
        state_map = {
            ScoopState.IDLE: ScoopStatus.STATE_IDLE,
            ScoopState.DEPLOYING: ScoopStatus.STATE_DEPLOYING,
            ScoopState.DEPLOYED: ScoopStatus.STATE_DEPLOYED,
            ScoopState.COLLECTING: ScoopStatus.STATE_COLLECTING,
            ScoopState.RETRACTING: ScoopStatus.STATE_RETRACTING,
            ScoopState.RETRACTED: ScoopStatus.STATE_RETRACTED,
            ScoopState.ERROR: ScoopStatus.STATE_ERROR,
        }
        msg.current_state = state_map.get(self.state, ScoopStatus.STATE_IDLE)
        
        msg.object_detected = self.object_detected
        msg.bin_full = self.bin_full
        msg.brush_current = self.brush_current
        msg.scoop_position = self.scoop_position
        
        msg.brush_motor_ok = self.brush_motor_ok
        msg.scoop_actuator_ok = self.scoop_actuator_ok
        msg.error_message = self.error_message
        
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScoopController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
