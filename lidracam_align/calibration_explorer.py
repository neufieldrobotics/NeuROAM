#!/usr/bin/env python3
"""
LiDAR-Camera Calibration Explorer

This script provides an interactive way to explore different calibration parameters
for aligning LiDAR point clouds with camera images. It generates a sequence of
visualizations with different parameter combinations to help find the optimal alignment.
"""

import os
import sys
import numpy as np
import cv2
import json
import argparse
import itertools
from pathlib import Path
import matplotlib
matplotlib.use('Agg')  # Use Agg backend for non-GUI environments
import matplotlib.pyplot as plt

# Import functions from align_tool.py
from align_tool import (
    load_camera_image, load_lidar_points, apply_transformation,
    project_points_to_image, save_calibration
)

def generate_parameter_combinations(param_ranges):
    """
    Generate combinations of parameters based on provided ranges.
    
    Args:
        param_ranges: Dictionary of parameter ranges
        
    Returns:
        List of parameter dictionaries
    """
    # Extract parameter names and values
    param_names = list(param_ranges.keys())
    param_values = [param_ranges[name] for name in param_names]
    
    # Generate all combinations
    combinations = list(itertools.product(*param_values))
    
    # Convert to list of dictionaries
    result = []
    for combo in combinations:
        params = {}
        for i, name in enumerate(param_names):
            params[name] = combo[i]
        result.append(params)
    
    return result

def create_grid_visualization(images, titles, output_file, grid_size=None):
    """
    Create a grid visualization of multiple images.
    
    Args:
        images: List of images
        titles: List of titles
        output_file: Output file path
        grid_size: Grid size (rows, cols)
    """
    n = len(images)
    
    if grid_size is None:
        # Calculate grid size
        cols = int(np.ceil(np.sqrt(n)))
        rows = int(np.ceil(n / cols))
    else:
        rows, cols = grid_size
    
    # Create figure
    plt.figure(figsize=(cols * 5, rows * 4))
    
    # Add each image to the grid
    for i in range(n):
        plt.subplot(rows, cols, i + 1)
        
        # Convert BGR to RGB for matplotlib
        if len(images[i].shape) == 3 and images[i].shape[2] == 3:
            plt.imshow(cv2.cvtColor(images[i], cv2.COLOR_BGR2RGB))
        else:
            plt.imshow(images[i], cmap='gray')
        
        plt.title(titles[i])
        plt.axis('off')
    
    # Save figure
    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    plt.close()
    
    print(f"Saved grid visualization to {output_file}")

def explore_calibration_parameters(image_path, points_path, output_dir, param_ranges=None):
    """
    Explore different calibration parameters and visualize the results.
    
    Args:
        image_path: Path to the camera image
        points_path: Path to the LiDAR point cloud
        output_dir: Output directory for visualizations
        param_ranges: Dictionary of parameter ranges
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Load data
    image = load_camera_image(image_path)
    points = load_lidar_points(points_path)
    
    if image is None or points is None:
        print("Error: Failed to load data")
        return
    
    # Default parameter ranges
    default_ranges = {
        "tx": [0.0, 1.0, 2.0],
        "ty": [0.0],
        "tz": [0.0],
        "rx": [0.0, 90.0, 180.0, 270.0],
        "ry": [0.0, 90.0, 180.0, 270.0],
        "rz": [0.0, 90.0, 180.0, 270.0],
        "scale": [0.1, 0.5, 1.0],
        "fx": [300.0, 500.0],
        "fy": [300.0, 500.0]
    }
    
    # Use provided parameter ranges or defaults
    if param_ranges is None:
        param_ranges = default_ranges
    else:
        # Ensure all parameters are present
        for key in default_ranges:
            if key not in param_ranges:
                param_ranges[key] = default_ranges[key]
    
    # Generate parameter combinations
    print("Generating parameter combinations...")
    combinations = generate_parameter_combinations(param_ranges)
    print(f"Generated {len(combinations)} parameter combinations")
    
    # Limit the number of combinations to avoid excessive processing
    max_combinations = 36
    if len(combinations) > max_combinations:
        print(f"Warning: Too many combinations ({len(combinations)}). Limiting to {max_combinations}.")
        combinations = combinations[:max_combinations]
    
    # Process each combination
    print("Processing parameter combinations...")
    results = []
    titles = []
    
    for i, params in enumerate(combinations):
        print(f"Processing combination {i+1}/{len(combinations)}: {params}")
        
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
        
        # Add to results
        results.append(result)
        
        # Create title
        title = f"tx={params['tx']}, ty={params['ty']}, tz={params['tz']}\n"
        title += f"rx={params['rx']}, ry={params['ry']}, rz={params['rz']}\n"
        title += f"scale={params['scale']}, fx={params['fx']}, fy={params['fy']}"
        titles.append(title)
    
    # Create grid visualization
    image_base = os.path.basename(image_path)
    points_base = os.path.basename(points_path)
    output_file = os.path.join(
        output_dir, 
        f"grid_{os.path.splitext(points_base)[0]}_{os.path.splitext(image_base)[0]}.png"
    )
    
    create_grid_visualization(results, titles, output_file)
    
    # Save individual results
    for i, (result, params) in enumerate(zip(results, combinations)):
        # Generate output filename
        output_file = os.path.join(
            output_dir, 
            f"aligned_{os.path.splitext(points_base)[0]}_{os.path.splitext(image_base)[0]}_{i:02d}.png"
        )
        
        # Save result
        cv2.imwrite(output_file, result)
        
        # Also save the parameters
        params_file = os.path.join(
            output_dir,
            f"params_{os.path.splitext(points_base)[0]}_{os.path.splitext(image_base)[0]}_{i:02d}.json"
        )
        save_calibration(params, params_file)
    
    print(f"Saved {len(results)} individual alignment visualizations")

def main():
    parser = argparse.ArgumentParser(description="LiDAR-Camera Calibration Explorer")
    parser.add_argument("--image", required=True, help="Path to the camera image")
    parser.add_argument("--points", required=True, help="Path to the LiDAR point cloud (.npy)")
    parser.add_argument("--output", default="calibration_explorer", help="Output directory")
    
    # Parameter ranges (optional)
    parser.add_argument("--tx-range", nargs='+', type=float, help="X translation range")
    parser.add_argument("--ty-range", nargs='+', type=float, help="Y translation range")
    parser.add_argument("--tz-range", nargs='+', type=float, help="Z translation range")
    parser.add_argument("--rx-range", nargs='+', type=float, help="X rotation range (degrees)")
    parser.add_argument("--ry-range", nargs='+', type=float, help="Y rotation range (degrees)")
    parser.add_argument("--rz-range", nargs='+', type=float, help="Z rotation range (degrees)")
    parser.add_argument("--scale-range", nargs='+', type=float, help="Scale factor range")
    parser.add_argument("--fx-range", nargs='+', type=float, help="X focal length range")
    parser.add_argument("--fy-range", nargs='+', type=float, help="Y focal length range")
    
    args = parser.parse_args()
    
    # Collect parameter ranges
    param_ranges = {}
    
    if args.tx_range:
        param_ranges["tx"] = args.tx_range
    if args.ty_range:
        param_ranges["ty"] = args.ty_range
    if args.tz_range:
        param_ranges["tz"] = args.tz_range
    if args.rx_range:
        param_ranges["rx"] = args.rx_range
    if args.ry_range:
        param_ranges["ry"] = args.ry_range
    if args.rz_range:
        param_ranges["rz"] = args.rz_range
    if args.scale_range:
        param_ranges["scale"] = args.scale_range
    if args.fx_range:
        param_ranges["fx"] = args.fx_range
    if args.fy_range:
        param_ranges["fy"] = args.fy_range
    
    # Explore calibration parameters
    explore_calibration_parameters(args.image, args.points, args.output, param_ranges)

if __name__ == "__main__":
    main()
