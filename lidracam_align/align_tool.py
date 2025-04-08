#!/usr/bin/env python3
"""
LiDAR-Camera Alignment Command-Line Tool

This script provides a command-line interface for aligning LiDAR point clouds with camera images.
It allows for manual adjustment of calibration parameters and saves the results.
"""

import os
import sys
import numpy as np
import cv2
import json
import argparse
import matplotlib
matplotlib.use('Agg')  # Use Agg backend for non-GUI environments
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

def load_camera_image(image_path):
    """
    Load a camera image from file.
    
    Args:
        image_path: Path to the image file
    
    Returns:
        The loaded image or None if loading failed
    """
    if not os.path.exists(image_path):
        print(f"Error: Image file not found: {image_path}")
        return None
    
    try:
        img = cv2.imread(image_path)
        if img is None:
            print(f"Error: Failed to load image: {image_path}")
            return None
        
        print(f"Loaded camera image: {image_path}, shape: {img.shape}")
        return img
    except Exception as e:
        print(f"Error loading image: {str(e)}")
        return None

def load_lidar_points(points_path):
    """
    Load LiDAR point cloud from file.
    
    Args:
        points_path: Path to the point cloud file (.npy)
    
    Returns:
        The loaded point cloud or None if loading failed
    """
    if not os.path.exists(points_path):
        print(f"Error: Point cloud file not found: {points_path}")
        return None
    
    try:
        # Check if it's a NumPy file (converted)
        if points_path.endswith('.npy'):
            # Load the NumPy array directly
            points = np.load(points_path)
            print(f"Loaded {len(points)} points from {points_path}")
            return points
        else:
            print(f"Error: Unsupported point cloud format: {points_path}")
            return None
    except Exception as e:
        print(f"Error loading point cloud: {str(e)}")
        return None

def apply_transformation(points, tx, ty, tz, rx, ry, rz, scale):
    """
    Apply transformation to point cloud.
    
    Args:
        points: Input point cloud
        tx, ty, tz: Translation parameters
        rx, ry, rz: Rotation parameters (in degrees)
        scale: Scale parameter
    
    Returns:
        Transformed point cloud
    """
    # Convert rotation angles from degrees to radians
    rx_rad = np.radians(rx)
    ry_rad = np.radians(ry)
    rz_rad = np.radians(rz)
    
    # Create rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx_rad), -np.sin(rx_rad)],
        [0, np.sin(rx_rad), np.cos(rx_rad)]
    ])
    
    Ry = np.array([
        [np.cos(ry_rad), 0, np.sin(ry_rad)],
        [0, 1, 0],
        [-np.sin(ry_rad), 0, np.cos(ry_rad)]
    ])
    
    Rz = np.array([
        [np.cos(rz_rad), -np.sin(rz_rad), 0],
        [np.sin(rz_rad), np.cos(rz_rad), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = Rz @ Ry @ Rx
    
    # Apply transformation
    transformed_points = np.copy(points)
    
    # Apply scale
    transformed_points = transformed_points * scale
    
    # Apply rotation
    transformed_points = transformed_points @ R.T
    
    # Apply translation
    transformed_points[:, 0] += tx
    transformed_points[:, 1] += ty
    transformed_points[:, 2] += tz
    
    return transformed_points

def project_points_to_image(points, image, fx=500, fy=500, cx=None, cy=None):
    """
    Project 3D points onto the image plane.
    
    Args:
        points: 3D points to project
        image: Target image
        fx, fy: Focal lengths
        cx, cy: Principal point (if None, use image center)
    
    Returns:
        Image with projected points
    """
    height, width = image.shape[:2]
    
    if cx is None:
        cx = width / 2
    if cy is None:
        cy = height / 2
    
    # Create a copy of the image for drawing
    result = image.copy()
    
    # Filter points in front of the camera (Z > 0)
    front_points = points[points[:, 0] > 0]
    
    if len(front_points) == 0:
        print("Warning: No points in front of the camera")
        return result
    
    # Project points to image coordinates
    x, y, z = front_points[:, 0], front_points[:, 1], front_points[:, 2]
    
    # Avoid division by zero
    x = np.maximum(x, 0.1)
    
    # Project to image coordinates
    u = fx * y / x + cx
    v = fy * z / x + cy
    
    # Calculate color based on distance
    distances = np.sqrt(np.sum(front_points**2, axis=1))
    max_dist = np.max(distances)
    
    # Filter points within image bounds
    valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, distances = u[valid], v[valid], distances[valid]
    
    # Draw points on the image
    for i in range(len(u)):
        # Color based on distance (red to blue)
        normalized_dist = distances[i] / max_dist
        color = (
            int(255 * (1 - normalized_dist)),  # B
            0,                                 # G
            int(255 * normalized_dist)         # R
        )
        
        cv2.circle(result, (int(u[i]), int(v[i])), 1, color, -1)
    
    return result

def save_calibration(params, output_file):
    """
    Save calibration parameters to a JSON file.
    
    Args:
        params: Calibration parameters
        output_file: Output file path
    """
    with open(output_file, 'w') as f:
        json.dump(params, f, indent=2)
    
    print(f"Saved calibration parameters to {output_file}")

def load_calibration(input_file):
    """
    Load calibration parameters from a JSON file.
    
    Args:
        input_file: Input file path
    
    Returns:
        Calibration parameters
    """
    with open(input_file, 'r') as f:
        params = json.load(f)
    
    print(f"Loaded calibration parameters from {input_file}")
    return params

def align_and_visualize(image_path, points_path, output_dir, params=None):
    """
    Align LiDAR points with camera image and visualize the result.
    
    Args:
        image_path: Path to the camera image
        points_path: Path to the LiDAR point cloud
        output_dir: Output directory for visualizations
        params: Calibration parameters (if None, use defaults)
    
    Returns:
        Path to the visualization image
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Load data
    image = load_camera_image(image_path)
    points = load_lidar_points(points_path)
    
    if image is None or points is None:
        print("Error: Failed to load data")
        return None
    
    # Default parameters
    default_params = {
        "tx": 0.0,
        "ty": 0.0,
        "tz": 0.0,
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "scale": 1.0,
        "fx": 500.0,
        "fy": 500.0
    }
    
    # Use provided parameters or defaults
    if params is None:
        params = default_params
    else:
        # Ensure all parameters are present
        for key in default_params:
            if key not in params:
                params[key] = default_params[key]
    
    # Apply transformation
    transformed_points = apply_transformation(
        points,
        params["tx"], params["ty"], params["tz"],
        params["rx"], params["ry"], params["rz"],
        params["scale"]
    )
    
    # Project points to image
    result = project_points_to_image(
        transformed_points, 
        image,
        params["fx"], params["fy"]
    )
    
    # Generate output filename
    image_base = os.path.basename(image_path)
    points_base = os.path.basename(points_path)
    output_file = os.path.join(
        output_dir, 
        f"aligned_{os.path.splitext(points_base)[0]}_{os.path.splitext(image_base)[0]}.png"
    )
    
    # Save result
    cv2.imwrite(output_file, result)
    print(f"Saved alignment visualization to {output_file}")
    
    # Also save the parameters
    params_file = os.path.join(
        output_dir,
        f"params_{os.path.splitext(points_base)[0]}_{os.path.splitext(image_base)[0]}.json"
    )
    save_calibration(params, params_file)
    
    return output_file

def main():
    parser = argparse.ArgumentParser(description="LiDAR-Camera Alignment Tool")
    parser.add_argument("--image", required=True, help="Path to the camera image")
    parser.add_argument("--points", required=True, help="Path to the LiDAR point cloud (.npy)")
    parser.add_argument("--output", default="aligned_output", help="Output directory")
    parser.add_argument("--params", help="Path to calibration parameters JSON file")
    
    # Transformation parameters
    parser.add_argument("--tx", type=float, default=0.0, help="X translation")
    parser.add_argument("--ty", type=float, default=0.0, help="Y translation")
    parser.add_argument("--tz", type=float, default=0.0, help="Z translation")
    parser.add_argument("--rx", type=float, default=0.0, help="X rotation (degrees)")
    parser.add_argument("--ry", type=float, default=0.0, help="Y rotation (degrees)")
    parser.add_argument("--rz", type=float, default=0.0, help="Z rotation (degrees)")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale factor")
    parser.add_argument("--fx", type=float, default=500.0, help="X focal length")
    parser.add_argument("--fy", type=float, default=500.0, help="Y focal length")
    
    args = parser.parse_args()
    
    # Load parameters from file if provided
    if args.params and os.path.exists(args.params):
        params = load_calibration(args.params)
    else:
        # Use command-line parameters
        params = {
            "tx": args.tx,
            "ty": args.ty,
            "tz": args.tz,
            "rx": args.rx,
            "ry": args.ry,
            "rz": args.rz,
            "scale": args.scale,
            "fx": args.fx,
            "fy": args.fy
        }
    
    # Align and visualize
    align_and_visualize(args.image, args.points, args.output, params)

if __name__ == "__main__":
    main()
