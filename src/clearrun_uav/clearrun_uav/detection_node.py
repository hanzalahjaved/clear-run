#!/usr/bin/env python3
"""
Clear-Run UAV Detection Node

This node performs real-time FOD detection using YOLOv11/v12 on the UAV's
camera feed (RGB and/or thermal IR). When debris is detected, it publishes
the detection to the visual servo node for precise localization.

Authors: Muhammad Hanzalah Javed, Aneeq
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, List, Tuple
from dataclasses import dataclass
import time

# YOLO imports
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[WARN] Ultralytics not installed. Running in mock mode.")

# Custom message imports
from clearrun_msgs.msg import FodLocation, FodArray


@dataclass
class Detection:
    """Data class for a single FOD detection."""
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    center: Tuple[int, int]
    source: int  # 0=RGB, 1=Thermal, 2=Fused


class FodDetectionNode(Node):
    """
    ROS 2 node for FOD detection using YOLO neural network.
    
    Subscribes to camera images (RGB and thermal), runs inference,
    and publishes detected FOD locations for the visual servo node.
    """
    
    # FOD class labels that our model is trained to detect
    FOD_CLASSES = [
        'bolt', 'nut', 'rivet', 'washer', 'screw',
        'wire', 'fragment', 'tool', 'luggage_tag', 
        'rubber', 'metal_piece', 'plastic', 'unknown_fod'
    ]
    
    def __init__(self):
        super().__init__('fod_detection_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'models/yolov11/fod_detector.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('use_thermal', True)
        self.declare_parameter('fusion_mode', 'late')  # 'early', 'late', 'rgb_only', 'thermal_only'
        self.declare_parameter('device', 'cuda:0')  # 'cuda:0', 'cpu'
        self.declare_parameter('publish_annotated', True)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.img_size = self.get_parameter('image_size').value
        self.use_thermal = self.get_parameter('use_thermal').value
        self.fusion_mode = self.get_parameter('fusion_mode').value
        self.device = self.get_parameter('device').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Load YOLO model
        self._load_model()
        
        # Detection ID counter
        self.detection_id = 0
        
        # Image buffers for fusion
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_thermal: Optional[np.ndarray] = None
        self.rgb_timestamp = None
        self.thermal_timestamp = None
        
        # QoS profile for image topics
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.rgb_callback,
            image_qos
        )
        
        if self.use_thermal:
            self.thermal_sub = self.create_subscription(
                Image,
                'camera/thermal',
                self.thermal_callback,
                image_qos
            )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            FodLocation,
            'detection/fod',
            10
        )
        
        self.detection_array_pub = self.create_publisher(
            FodArray,
            'detection/fod_array',
            10
        )
        
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                'detection/annotated',
                image_qos
            )
        
        # Performance metrics
        self.inference_times = []
        
        self.get_logger().info(f'FOD Detection Node initialized')
        self.get_logger().info(f'  Model: {self.model_path}')
        self.get_logger().info(f'  Device: {self.device}')
        self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'  Fusion mode: {self.fusion_mode}')
    
    def _load_model(self):
        """Load the YOLO model for inference."""
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                self.model.to(self.device)
                self.get_logger().info(f'YOLO model loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.model = None
        else:
            self.model = None
            self.get_logger().warn('Running in mock mode - no actual detection')
    
    def rgb_callback(self, msg: Image):
        """Handle incoming RGB camera images."""
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.rgb_timestamp = msg.header.stamp
            
            # Process based on fusion mode
            if self.fusion_mode in ['rgb_only', 'late']:
                self._run_detection(self.latest_rgb, source=FodLocation.SOURCE_RGB)
            elif self.fusion_mode == 'early' and self.latest_thermal is not None:
                self._run_fused_detection()
                
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')
    
    def thermal_callback(self, msg: Image):
        """Handle incoming thermal camera images."""
        try:
            # Thermal images typically come as mono8 or mono16
            self.latest_thermal = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            self.thermal_timestamp = msg.header.stamp
            
            # Process based on fusion mode
            if self.fusion_mode == 'thermal_only':
                # Convert to 3-channel for YOLO
                thermal_rgb = cv2.cvtColor(self.latest_thermal, cv2.COLOR_GRAY2BGR)
                self._run_detection(thermal_rgb, source=FodLocation.SOURCE_THERMAL)
            elif self.fusion_mode == 'late':
                thermal_rgb = cv2.cvtColor(self.latest_thermal, cv2.COLOR_GRAY2BGR)
                self._run_detection(thermal_rgb, source=FodLocation.SOURCE_THERMAL)
            elif self.fusion_mode == 'early' and self.latest_rgb is not None:
                self._run_fused_detection()
                
        except Exception as e:
            self.get_logger().error(f'Thermal callback error: {e}')
    
    def _run_fused_detection(self):
        """Run detection on fused RGB+Thermal image."""
        if self.latest_rgb is None or self.latest_thermal is None:
            return
        
        # Early fusion: Stack RGB and thermal as 4-channel input
        # For standard YOLO, we'll use a simple fusion strategy
        thermal_3ch = cv2.cvtColor(self.latest_thermal, cv2.COLOR_GRAY2BGR)
        
        # Resize thermal to match RGB if needed
        if thermal_3ch.shape[:2] != self.latest_rgb.shape[:2]:
            thermal_3ch = cv2.resize(thermal_3ch, 
                                     (self.latest_rgb.shape[1], self.latest_rgb.shape[0]))
        
        # Simple fusion: weighted average (can be improved with learned fusion)
        fused = cv2.addWeighted(self.latest_rgb, 0.7, thermal_3ch, 0.3, 0)
        
        self._run_detection(fused, source=FodLocation.SOURCE_FUSED)
    
    def _run_detection(self, image: np.ndarray, source: int):
        """
        Run YOLO inference on the image and publish detections.
        
        Args:
            image: BGR image array
            source: Detection source (RGB, thermal, or fused)
        """
        if self.model is None:
            # Mock detection for testing without GPU
            self._publish_mock_detection(image, source)
            return
        
        start_time = time.time()
        
        # Run inference
        results = self.model.predict(
            source=image,
            conf=self.conf_threshold,
            iou=self.nms_threshold,
            imgsz=self.img_size,
            verbose=False
        )
        
        inference_time = time.time() - start_time
        self.inference_times.append(inference_time)
        
        # Process results
        detections = []
        annotated_image = image.copy() if self.publish_annotated else None
        
        for result in results:
            if result.boxes is None:
                continue
                
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                
                # Get class name
                class_name = result.names.get(cls_id, 'unknown_fod')
                
                # Skip if not a FOD class
                if class_name not in self.FOD_CLASSES:
                    continue
                
                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                bbox = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
                center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                
                detection = Detection(
                    class_name=class_name,
                    confidence=conf,
                    bbox=bbox,
                    center=center,
                    source=source
                )
                detections.append(detection)
                
                # Draw on annotated image
                if annotated_image is not None:
                    self._draw_detection(annotated_image, detection)
        
        # Publish detections
        self._publish_detections(detections, image.shape, source)
        
        # Publish annotated image
        if annotated_image is not None and len(detections) > 0:
            self._publish_annotated_image(annotated_image)
        
        # Log performance (every 100 frames)
        if len(self.inference_times) % 100 == 0:
            avg_time = np.mean(self.inference_times[-100:])
            fps = 1.0 / avg_time if avg_time > 0 else 0
            self.get_logger().info(f'Detection FPS: {fps:.1f} ({avg_time*1000:.1f}ms)')
    
    def _draw_detection(self, image: np.ndarray, detection: Detection):
        """Draw detection bounding box and label on image."""
        x, y, w, h = detection.bbox
        
        # Color based on source
        colors = {0: (0, 255, 0), 1: (255, 0, 0), 2: (0, 255, 255)}
        color = colors.get(detection.source, (255, 255, 255))
        
        # Draw box
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        
        # Draw label
        label = f'{detection.class_name}: {detection.confidence:.2f}'
        cv2.putText(image, label, (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw center point
        cv2.circle(image, detection.center, 5, (0, 0, 255), -1)
    
    def _publish_detections(self, detections: List[Detection], 
                           image_shape: Tuple, source: int):
        """Publish detected FOD locations."""
        if not detections:
            return
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'uav_camera_optical_frame'
        
        fod_array = FodArray()
        fod_array.header = header
        
        for det in detections:
            self.detection_id += 1
            
            fod_msg = FodLocation()
            fod_msg.header = header
            fod_msg.fod_id = self.detection_id
            fod_msg.object_class = det.class_name
            fod_msg.confidence = det.confidence
            fod_msg.detection_source = det.source
            
            # Bounding box
            fod_msg.bbox_x = det.bbox[0]
            fod_msg.bbox_y = det.bbox[1]
            fod_msg.bbox_width = det.bbox[2]
            fod_msg.bbox_height = det.bbox[3]
            
            # Estimate size based on bbox and assumed altitude
            # This is a rough estimate - visual servo will refine
            fod_msg.estimated_size = float(max(det.bbox[2], det.bbox[3]) * 0.1)
            
            # GPS will be filled by visual servo node after centering
            fod_msg.latitude = 0.0
            fod_msg.longitude = 0.0
            fod_msg.altitude = 0.0
            fod_msg.is_centered = False
            fod_msg.is_collected = False
            fod_msg.requires_manual = False
            
            # Publish individual detection
            self.detection_pub.publish(fod_msg)
            fod_array.detections.append(fod_msg)
            
            self.get_logger().info(
                f'FOD detected: {det.class_name} ({det.confidence:.2f}) '
                f'at pixel ({det.center[0]}, {det.center[1]})'
            )
        
        # Publish array of all detections
        self.detection_array_pub.publish(fod_array)
    
    def _publish_mock_detection(self, image: np.ndarray, source: int):
        """Publish mock detection for testing without YOLO."""
        # Simulate random detections for testing
        import random
        if random.random() < 0.1:  # 10% chance of detection
            detection = Detection(
                class_name=random.choice(self.FOD_CLASSES),
                confidence=random.uniform(0.5, 0.99),
                bbox=(random.randint(100, 400), random.randint(100, 400), 50, 50),
                center=(random.randint(125, 425), random.randint(125, 425)),
                source=source
            )
            self._publish_detections([detection], image.shape, source)
    
    def _publish_annotated_image(self, image: np.ndarray):
        """Publish the annotated image with detection overlays."""
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'uav_camera_optical_frame'
            self.annotated_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FodDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
