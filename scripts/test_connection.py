#!/usr/bin/env python3
"""
Clear-Run Connection Test Tool

Tests communication between system components:
- ROS 2 topic connectivity
- MAVROS connection to flight controllers
- Inter-robot communication (UAV <-> UGV)

Usage:
    python3 test_connection.py --all
    python3 test_connection.py --mavros
    python3 test_connection.py --ros

Authors: Muhammad Hanzalah Javed, Aneeq
"""

import argparse
import subprocess
import time
import sys

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import NavSatFix
    from mavros_msgs.msg import State
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# Colors for terminal output
class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    END = '\033[0m'
    BOLD = '\033[1m'


def print_header(text):
    print(f"\n{Colors.BOLD}{Colors.CYAN}{'='*60}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}  {text}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}{'='*60}{Colors.END}\n")


def print_pass(text):
    print(f"{Colors.GREEN}[PASS]{Colors.END} {text}")


def print_fail(text):
    print(f"{Colors.RED}[FAIL]{Colors.END} {text}")


def print_warn(text):
    print(f"{Colors.YELLOW}[WARN]{Colors.END} {text}")


def print_info(text):
    print(f"{Colors.CYAN}[INFO]{Colors.END} {text}")


def check_ros2_daemon():
    """Check if ROS 2 daemon is running."""
    try:
        result = subprocess.run(
            ['ros2', 'daemon', 'status'],
            capture_output=True, text=True, timeout=5
        )
        if 'running' in result.stdout.lower():
            print_pass("ROS 2 daemon is running")
            return True
        else:
            print_warn("ROS 2 daemon not running, starting...")
            subprocess.run(['ros2', 'daemon', 'start'], timeout=10)
            return True
    except Exception as e:
        print_fail(f"ROS 2 daemon check failed: {e}")
        return False


def check_topics():
    """Check for active ROS 2 topics."""
    print_info("Checking ROS 2 topics...")
    
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=10
        )
        topics = result.stdout.strip().split('\n')
        topics = [t for t in topics if t]  # Remove empty strings
        
        print_info(f"Found {len(topics)} active topics")
        
        # Check for key topics
        key_topics = {
            '/uav/mavros/state': 'UAV MAVROS',
            '/ugv/mavros/state': 'UGV MAVROS',
            '/uav/detection/fod': 'FOD Detection',
            '/ugv/fod_target': 'UGV Target',
            '/uav/camera/image_raw': 'UAV Camera',
        }
        
        for topic, name in key_topics.items():
            if topic in topics:
                print_pass(f"{name}: {topic}")
            else:
                print_warn(f"{name} not found: {topic}")
        
        return len(topics) > 0
    except Exception as e:
        print_fail(f"Topic check failed: {e}")
        return False


def check_nodes():
    """Check for active ROS 2 nodes."""
    print_info("Checking ROS 2 nodes...")
    
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True, text=True, timeout=10
        )
        nodes = result.stdout.strip().split('\n')
        nodes = [n for n in nodes if n]
        
        print_info(f"Found {len(nodes)} active nodes")
        
        # Check for key nodes
        key_nodes = ['detection_node', 'visual_servo', 'fod_retriever', 'scoop_controller']
        
        for key_node in key_nodes:
            found = any(key_node in n for n in nodes)
            if found:
                print_pass(f"Node active: {key_node}")
            else:
                print_warn(f"Node not found: {key_node}")
        
        return len(nodes) > 0
    except Exception as e:
        print_fail(f"Node check failed: {e}")
        return False


def check_mavros_connection(namespace: str, timeout: float = 5.0):
    """Check MAVROS connection to flight controller."""
    if not ROS_AVAILABLE:
        print_fail("ROS 2 Python libraries not available")
        return False
    
    print_info(f"Checking MAVROS connection for {namespace}...")
    
    class MavrosChecker(Node):
        def __init__(self, ns):
            super().__init__(f'mavros_checker_{ns.replace("/", "_")}')
            self.connected = False
            self.armed = False
            self.mode = "UNKNOWN"
            
            self.sub = self.create_subscription(
                State,
                f'{ns}/mavros/state',
                self.state_callback,
                10
            )
        
        def state_callback(self, msg):
            self.connected = msg.connected
            self.armed = msg.armed
            self.mode = msg.mode
    
    rclpy.init()
    node = MavrosChecker(namespace)
    
    start = time.time()
    while (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.connected:
            break
    
    if node.connected:
        print_pass(f"{namespace} MAVROS connected")
        print_info(f"  Mode: {node.mode}, Armed: {node.armed}")
        result = True
    else:
        print_fail(f"{namespace} MAVROS not connected")
        result = False
    
    node.destroy_node()
    rclpy.shutdown()
    
    return result


def check_gps(namespace: str, timeout: float = 5.0):
    """Check GPS fix."""
    if not ROS_AVAILABLE:
        return False
    
    print_info(f"Checking GPS for {namespace}...")
    
    class GpsChecker(Node):
        def __init__(self, ns):
            super().__init__(f'gps_checker_{ns.replace("/", "_")}')
            self.has_fix = False
            self.lat = 0.0
            self.lon = 0.0
            
            self.sub = self.create_subscription(
                NavSatFix,
                f'{ns}/mavros/global_position/global',
                self.gps_callback,
                10
            )
        
        def gps_callback(self, msg):
            self.has_fix = True
            self.lat = msg.latitude
            self.lon = msg.longitude
    
    rclpy.init()
    node = GpsChecker(namespace)
    
    start = time.time()
    while (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.has_fix:
            break
    
    if node.has_fix:
        print_pass(f"{namespace} GPS: ({node.lat:.6f}, {node.lon:.6f})")
        result = True
    else:
        print_warn(f"{namespace} GPS fix not available")
        result = False
    
    node.destroy_node()
    rclpy.shutdown()
    
    return result


def test_ros_communication():
    """Test basic ROS 2 communication."""
    print_header("ROS 2 Communication Test")
    
    results = []
    results.append(check_ros2_daemon())
    results.append(check_topics())
    results.append(check_nodes())
    
    return all(results)


def test_mavros():
    """Test MAVROS connections."""
    print_header("MAVROS Connection Test")
    
    results = []
    
    # Test UAV
    results.append(check_mavros_connection('/uav'))
    
    # Test UGV
    results.append(check_mavros_connection('/ugv'))
    
    return all(results)


def test_gps():
    """Test GPS connectivity."""
    print_header("GPS Test")
    
    results = []
    results.append(check_gps('/uav'))
    results.append(check_gps('/ugv'))
    
    return all(results)


def test_all():
    """Run all tests."""
    print_header("CLEAR-RUN CONNECTION TEST")
    
    results = {
        'ROS 2': test_ros_communication(),
        'MAVROS': test_mavros() if ROS_AVAILABLE else False,
        'GPS': test_gps() if ROS_AVAILABLE else False,
    }
    
    print_header("TEST SUMMARY")
    
    all_passed = True
    for test, passed in results.items():
        if passed:
            print_pass(f"{test} tests passed")
        else:
            print_fail(f"{test} tests failed")
            all_passed = False
    
    return all_passed


def main():
    parser = argparse.ArgumentParser(
        description='Clear-Run connection test tool'
    )
    parser.add_argument('--all', '-a', action='store_true',
                       help='Run all tests')
    parser.add_argument('--ros', '-r', action='store_true',
                       help='Test ROS 2 communication')
    parser.add_argument('--mavros', '-m', action='store_true',
                       help='Test MAVROS connections')
    parser.add_argument('--gps', '-g', action='store_true',
                       help='Test GPS connectivity')
    
    args = parser.parse_args()
    
    # Default to all if no specific test selected
    if not any([args.all, args.ros, args.mavros, args.gps]):
        args.all = True
    
    success = True
    
    if args.all:
        success = test_all()
    else:
        if args.ros:
            success &= test_ros_communication()
        if args.mavros:
            success &= test_mavros()
        if args.gps:
            success &= test_gps()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
