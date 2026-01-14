#!/usr/bin/env python3
"""
Clear-Run Camera Calibration Tool

Calibrates the UAV camera for accurate visual measurements.
Uses a checkerboard pattern to compute camera intrinsics.

Usage:
    python3 calibrate_camera.py --input /path/to/images --size 9x6 --square 0.025
    
Authors: Muhammad Hanzalah Javed, Aneeq
"""

import argparse
import cv2
import numpy as np
import glob
import os
import json
from datetime import datetime


def find_checkerboard_corners(image_paths: list, pattern_size: tuple, 
                               show_corners: bool = False) -> tuple:
    """
    Find checkerboard corners in calibration images.
    
    Args:
        image_paths: List of paths to calibration images
        pattern_size: Tuple of (columns, rows) of inner corners
        show_corners: Whether to display detected corners
        
    Returns:
        Tuple of (object_points, image_points, image_size)
    """
    # Prepare object points (0,0,0), (1,0,0), (2,0,0), ...
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    
    object_points = []  # 3D points in real world space
    image_points = []   # 2D points in image plane
    image_size = None
    
    print(f"Processing {len(image_paths)} images...")
    
    for i, path in enumerate(image_paths):
        img = cv2.imread(path)
        if img is None:
            print(f"  [!] Could not read: {path}")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if image_size is None:
            image_size = gray.shape[::-1]
        
        # Find corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        if ret:
            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            object_points.append(objp)
            image_points.append(corners_refined)
            
            print(f"  [✓] {os.path.basename(path)}")
            
            if show_corners:
                cv2.drawChessboardCorners(img, pattern_size, corners_refined, ret)
                cv2.imshow('Corners', cv2.resize(img, (960, 540)))
                cv2.waitKey(500)
        else:
            print(f"  [✗] {os.path.basename(path)} - No corners found")
    
    if show_corners:
        cv2.destroyAllWindows()
    
    return object_points, image_points, image_size


def calibrate_camera(object_points: list, image_points: list, 
                    image_size: tuple) -> dict:
    """
    Perform camera calibration.
    
    Args:
        object_points: List of 3D object points
        image_points: List of 2D image points
        image_size: Image dimensions (width, height)
        
    Returns:
        Dictionary containing calibration parameters
    """
    print("\nCalibrating camera...")
    
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    
    # Calculate reprojection error
    mean_error = 0
    for i in range(len(object_points)):
        imgpoints2, _ = cv2.projectPoints(
            object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    
    mean_error /= len(object_points)
    
    calibration = {
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coefficients': dist_coeffs.tolist(),
        'image_width': image_size[0],
        'image_height': image_size[1],
        'reprojection_error': mean_error,
        'num_images': len(object_points),
        'calibration_date': datetime.now().isoformat(),
        
        # Derived parameters
        'fx': camera_matrix[0, 0],
        'fy': camera_matrix[1, 1],
        'cx': camera_matrix[0, 2],
        'cy': camera_matrix[1, 2],
    }
    
    return calibration


def save_calibration(calibration: dict, output_path: str, 
                    square_size: float = None):
    """Save calibration to JSON and YAML files."""
    
    if square_size:
        calibration['square_size_meters'] = square_size
    
    # Save JSON
    json_path = output_path + '.json'
    with open(json_path, 'w') as f:
        json.dump(calibration, f, indent=2)
    print(f"Saved: {json_path}")
    
    # Save YAML (for ROS)
    yaml_path = output_path + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write("# Camera calibration parameters\n")
        f.write(f"# Generated: {calibration['calibration_date']}\n\n")
        f.write(f"image_width: {calibration['image_width']}\n")
        f.write(f"image_height: {calibration['image_height']}\n\n")
        f.write("camera_matrix:\n")
        f.write(f"  rows: 3\n")
        f.write(f"  cols: 3\n")
        f.write(f"  data: {calibration['camera_matrix']}\n\n")
        f.write("distortion_coefficients:\n")
        f.write(f"  rows: 1\n")
        f.write(f"  cols: 5\n")
        f.write(f"  data: {calibration['distortion_coefficients'][0]}\n")
    print(f"Saved: {yaml_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Camera calibration tool for Clear-Run'
    )
    parser.add_argument(
        '--input', '-i', required=True,
        help='Path to directory containing calibration images'
    )
    parser.add_argument(
        '--size', '-s', default='9x6',
        help='Checkerboard pattern size (columns x rows of inner corners)'
    )
    parser.add_argument(
        '--square', '-q', type=float, default=0.025,
        help='Square size in meters (default: 0.025)'
    )
    parser.add_argument(
        '--output', '-o', default='camera_calibration',
        help='Output file basename (without extension)'
    )
    parser.add_argument(
        '--show', action='store_true',
        help='Show detected corners'
    )
    
    args = parser.parse_args()
    
    # Parse pattern size
    pattern_size = tuple(map(int, args.size.split('x')))
    print(f"Pattern size: {pattern_size[0]} x {pattern_size[1]}")
    print(f"Square size: {args.square}m")
    
    # Find calibration images
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    image_paths = []
    for ext in image_extensions:
        image_paths.extend(glob.glob(os.path.join(args.input, ext)))
    
    if not image_paths:
        print(f"Error: No images found in {args.input}")
        return 1
    
    print(f"Found {len(image_paths)} images")
    
    # Find corners
    object_points, image_points, image_size = find_checkerboard_corners(
        image_paths, pattern_size, args.show
    )
    
    if len(object_points) < 10:
        print(f"\nWarning: Only {len(object_points)} valid images found.")
        print("Recommend at least 10-20 images for good calibration.")
    
    if len(object_points) < 3:
        print("Error: Need at least 3 valid images for calibration")
        return 1
    
    # Scale object points by square size
    for i in range(len(object_points)):
        object_points[i] *= args.square
    
    # Calibrate
    calibration = calibrate_camera(object_points, image_points, image_size)
    
    # Print results
    print("\n" + "=" * 50)
    print("CALIBRATION RESULTS")
    print("=" * 50)
    print(f"Images used: {calibration['num_images']}")
    print(f"Image size: {calibration['image_width']} x {calibration['image_height']}")
    print(f"Reprojection error: {calibration['reprojection_error']:.4f} pixels")
    print(f"\nFocal length: fx={calibration['fx']:.2f}, fy={calibration['fy']:.2f}")
    print(f"Principal point: cx={calibration['cx']:.2f}, cy={calibration['cy']:.2f}")
    print("=" * 50)
    
    # Save
    save_calibration(calibration, args.output, args.square)
    
    print("\nCalibration complete!")
    return 0


if __name__ == '__main__':
    exit(main())
