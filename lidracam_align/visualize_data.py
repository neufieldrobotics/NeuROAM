#!/usr/bin/env python3
"""
Script to visualize the converted LiDAR and camera data.
This script will generate visualizations of the data and save them as images.
"""

import os
import sys
import numpy as np
import cv2
# Set matplotlib to use 'Agg' backend (non-GUI)
import matplotlib
matplotlib.use('Agg')  # This must be done before importing pyplot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
from pathlib import Path

def visualize_camera_image(image_path, output_dir=None):
    """
    Visualize a camera image and save the visualization.
    
    Args:
        image_path: Path to the image file
        output_dir: Directory to save the visualization (default: same as input)
    
    Returns:
        Path to the visualization image
    """
    if output_dir is None:
        output_dir = os.path.dirname(image_path)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate output filename
    base_name = os.path.basename(image_path)
    name_without_ext = os.path.splitext(base_name)[0]
    output_file = os.path.join(output_dir, f"{name_without_ext}_vis.png")
    
    print(f"Visualizing {image_path} to {output_file}")
    
    try:
        # Read the image
        img = cv2.imread(image_path)
        if img is None:
            print(f"  Error reading image: {image_path}")
            return None
        
        # Convert BGR to RGB for matplotlib
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Create a figure
        plt.figure(figsize=(10, 6))
        plt.imshow(img_rgb)
        plt.title(f"Camera Image: {os.path.basename(image_path)}")
        plt.axis('off')
        
        # Save the figure
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"  Successfully visualized camera image to {output_file}")
        return output_file
    
    except Exception as e:
        print(f"  Error visualizing {image_path}: {e}")
        return None

def visualize_lidar_points(points_path, output_dir=None):
    """
    Visualize a LiDAR point cloud and save the visualization.
    
    Args:
        points_path: Path to the point cloud file (.npy)
        output_dir: Directory to save the visualization (default: same as input)
    
    Returns:
        Path to the visualization image
    """
    if output_dir is None:
        output_dir = os.path.dirname(points_path)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate output filename
    base_name = os.path.basename(points_path)
    name_without_ext = os.path.splitext(base_name)[0]
    output_file = os.path.join(output_dir, f"{name_without_ext}_vis.png")
    
    print(f"Visualizing {points_path} to {output_file}")
    
    try:
        # Load the point cloud
        points = np.load(points_path)
        
        # Create a figure
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot the points
        # Subsample if there are too many points
        max_points = 10000
        if len(points) > max_points:
            indices = np.random.choice(len(points), max_points, replace=False)
            points_subset = points[indices]
        else:
            points_subset = points
        
        # Calculate point color based on distance from origin
        distances = np.sqrt(np.sum(points_subset**2, axis=1))
        
        # Plot the points
        scatter = ax.scatter(
            points_subset[:, 0],
            points_subset[:, 1],
            points_subset[:, 2],
            c=distances,
            cmap='viridis',
            s=1
        )
        
        # Add a colorbar
        cbar = plt.colorbar(scatter)
        cbar.set_label('Distance from origin (m)')
        
        # Set labels and title
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f"LiDAR Point Cloud: {os.path.basename(points_path)}")
        
        # Set equal aspect ratio
        ax.set_box_aspect([1, 1, 0.5])
        
        # Save the figure
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"  Successfully visualized point cloud to {output_file}")
        
        # Also create top-down view
        top_down_file = os.path.join(output_dir, f"{name_without_ext}_top_down.png")
        
        plt.figure(figsize=(10, 10))
        plt.scatter(
            points_subset[:, 0],
            points_subset[:, 1],
            c=distances,
            cmap='viridis',
            s=1
        )
        plt.colorbar(label='Distance from origin (m)')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(f"LiDAR Point Cloud (Top-Down View): {os.path.basename(points_path)}")
        plt.axis('equal')
        plt.grid(True)
        
        # Save the figure
        plt.savefig(top_down_file, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"  Successfully created top-down view to {top_down_file}")
        
        return output_file
    
    except Exception as e:
        print(f"  Error visualizing {points_path}: {e}")
        return None

def visualize_lidar_camera_overlay(points_path, image_path, output_dir=None):
    """
    Create a simple overlay of LiDAR points on a camera image.
    This is a placeholder for a more sophisticated alignment visualization.
    
    Args:
        points_path: Path to the point cloud file (.npy)
        image_path: Path to the image file
        output_dir: Directory to save the visualization (default: same as input)
    
    Returns:
        Path to the visualization image
    """
    if output_dir is None:
        output_dir = os.path.dirname(image_path)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate output filename
    points_base = os.path.basename(points_path)
    image_base = os.path.basename(image_path)
    output_file = os.path.join(output_dir, f"overlay_{os.path.splitext(points_base)[0]}_{os.path.splitext(image_base)[0]}.png")
    
    print(f"Creating overlay of {points_path} on {image_path} to {output_file}")
    
    try:
        # Load the point cloud
        points = np.load(points_path)
        
        # Load the image
        img = cv2.imread(image_path)
        if img is None:
            print(f"  Error reading image: {image_path}")
            return None
        
        # Convert BGR to RGB for matplotlib
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Get image dimensions
        height, width = img.shape[:2]
        
        # Create a figure
        plt.figure(figsize=(12, 8))
        
        # Display the image
        plt.imshow(img_rgb)
        
        # Project LiDAR points onto the image (simple projection for demonstration)
        # This is a placeholder - a real implementation would use camera calibration
        # and proper 3D-to-2D projection
        
        # Subsample if there are too many points
        max_points = 5000
        if len(points) > max_points:
            indices = np.random.choice(len(points), max_points, replace=False)
            points_subset = points[indices]
        else:
            points_subset = points
        
        # Simple projection: assume points in front of camera (x > 0)
        # and project using simple pinhole camera model
        front_points = points_subset[points_subset[:, 0] > 0]
        
        if len(front_points) > 0:
            # Simple projection (this is just for visualization, not accurate)
            focal_length = width / 2  # Approximate focal length
            cx, cy = width / 2, height / 2  # Image center
            
            # Project points
            x, y, z = front_points[:, 0], front_points[:, 1], front_points[:, 2]
            
            # Avoid division by zero
            z = np.maximum(z, 0.1)
            
            # Project to image coordinates
            u = focal_length * y / x + cx
            v = focal_length * z / x + cy
            
            # Filter points within image bounds
            valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
            u, v = u[valid], v[valid]
            
            # Calculate color based on distance
            distances = np.sqrt(np.sum(front_points[valid]**2, axis=1))
            
            # Plot the projected points
            plt.scatter(u, v, c=distances, cmap='jet', alpha=0.5, s=1)
            plt.colorbar(label='Distance from origin (m)')
        
        plt.title(f"LiDAR-Camera Overlay (Simple Projection)")
        plt.axis('off')
        
        # Save the figure
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"  Successfully created overlay to {output_file}")
        return output_file
    
    except Exception as e:
        print(f"  Error creating overlay: {e}")
        return None

def main():
    # Get the extracted data directory
    extracted_data_dir = "extracted_data"
    
    if not os.path.exists(extracted_data_dir):
        print(f"Error: Directory {extracted_data_dir} not found")
        return
    
    # Find all converted data
    camera_images = []
    lidar_points = []
    
    for root, dirs, files in os.walk(extracted_data_dir):
        for file in files:
            if file.endswith('.png') and 'converted' in root:
                camera_images.append(os.path.join(root, file))
            elif file.endswith('.npy') and 'converted' in root:
                lidar_points.append(os.path.join(root, file))
    
    print(f"Found {len(camera_images)} camera images and {len(lidar_points)} LiDAR point clouds")
    
    # Create a visualizations directory
    vis_dir = os.path.join(extracted_data_dir, "visualizations")
    os.makedirs(vis_dir, exist_ok=True)
    
    # Visualize camera images
    for image_path in camera_images:
        visualize_camera_image(image_path, vis_dir)
    
    # Visualize LiDAR point clouds
    for points_path in lidar_points:
        visualize_lidar_points(points_path, vis_dir)
    
    # Create overlays (if we have both camera images and LiDAR points)
    if camera_images and lidar_points:
        # Use the first point cloud and camera image for demonstration
        visualize_lidar_camera_overlay(lidar_points[0], camera_images[0], vis_dir)

if __name__ == "__main__":
    main()
