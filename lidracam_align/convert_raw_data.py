#!/usr/bin/env python3
"""
Script to convert raw binary data extracted from ROS bag files into usable formats
for the LiDAR-Camera alignment GUI.

This script attempts to parse the raw binary data and convert it to:
- PNG images for camera data
- Numpy arrays for LiDAR point clouds
"""

import os
import sys
import numpy as np
import cv2
import struct
import json
from pathlib import Path

def convert_raw_camera_image(raw_file_path, output_dir=None):
    """
    Attempt to convert a raw camera image binary file to a PNG image.
    
    Args:
        raw_file_path: Path to the raw binary file
        output_dir: Directory to save the converted image (default: same as input)
    
    Returns:
        Path to the converted image or None if conversion failed
    """
    if output_dir is None:
        output_dir = os.path.dirname(raw_file_path)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate output filename
    base_name = os.path.basename(raw_file_path)
    name_without_ext = os.path.splitext(base_name)[0]
    output_file = os.path.join(output_dir, f"{name_without_ext}.png")
    
    print(f"Converting {raw_file_path} to {output_file}")
    
    try:
        # Read the raw binary data
        with open(raw_file_path, 'rb') as f:
            data = f.read()
        
        # Try different approaches to parse the image data
        
        # Approach 1: Try to find image dimensions in the header
        # ROS image messages typically have a header followed by image data
        # We'll look for common image dimensions (e.g., 640x480, 1280x720)
        common_dimensions = [
            (640, 480),  # VGA
            (800, 600),  # SVGA
            (1024, 768),  # XGA
            (1280, 720),  # HD
            (1920, 1080)  # Full HD
        ]
        
        # Find the image data section (after the header)
        # This is a heuristic approach - we look for a large continuous block of data
        header_size_guesses = [100, 200, 300, 400, 500]
        
        success = False
        
        for header_size in header_size_guesses:
            if header_size >= len(data):
                continue
                
            image_data = data[header_size:]
            
            for width, height in common_dimensions:
                # Try RGB (3 channels)
                expected_size = width * height * 3
                if len(image_data) >= expected_size:
                    try:
                        img_array = np.frombuffer(image_data[:expected_size], dtype=np.uint8)
                        img = img_array.reshape((height, width, 3))
                        
                        # Check if the image looks valid
                        if np.mean(img) > 5 and np.std(img) > 5:  # Simple heuristic
                            cv2.imwrite(output_file, img)
                            print(f"  Successfully converted to {width}x{height} RGB image")
                            return output_file
                    except Exception as e:
                        print(f"  Failed RGB conversion at {width}x{height}: {e}")
                
                # Try grayscale (1 channel)
                expected_size = width * height
                if len(image_data) >= expected_size:
                    try:
                        img_array = np.frombuffer(image_data[:expected_size], dtype=np.uint8)
                        img = img_array.reshape((height, width))
                        
                        # Check if the image looks valid
                        if np.mean(img) > 5 and np.std(img) > 5:  # Simple heuristic
                            cv2.imwrite(output_file, img)
                            print(f"  Successfully converted to {width}x{height} grayscale image")
                            return output_file
                    except Exception as e:
                        print(f"  Failed grayscale conversion at {width}x{height}: {e}")
        
        # If we get here, all approaches failed
        print(f"  Failed to convert {raw_file_path} to image")
        return None
    
    except Exception as e:
        print(f"  Error converting {raw_file_path}: {e}")
        return None

def convert_raw_lidar_data(raw_file_path, output_dir=None):
    """
    Attempt to convert a raw LiDAR binary file to a numpy array.
    
    Args:
        raw_file_path: Path to the raw binary file
        output_dir: Directory to save the converted point cloud (default: same as input)
    
    Returns:
        Path to the converted point cloud or None if conversion failed
    """
    if output_dir is None:
        output_dir = os.path.dirname(raw_file_path)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate output filename
    base_name = os.path.basename(raw_file_path)
    name_without_ext = os.path.splitext(base_name)[0]
    output_file = os.path.join(output_dir, f"{name_without_ext}.npy")
    
    print(f"Converting {raw_file_path} to {output_file}")
    
    try:
        # Read the raw binary data
        with open(raw_file_path, 'rb') as f:
            data = f.read()
        
        # Check if metadata file exists
        metadata_path = os.path.join(os.path.dirname(raw_file_path), "metadata.txt")
        point_step = 32  # Default point step (common for PointCloud2 messages with x,y,z,intensity)
        
        if os.path.exists(metadata_path):
            # Parse metadata
            metadata = {}
            with open(metadata_path, 'r') as f:
                for line in f:
                    if ':' in line:
                        key, value = line.strip().split(':', 1)
                        metadata[key.strip()] = value.strip()
            
            # Get point step if available
            if 'point_step' in metadata:
                try:
                    point_step = int(metadata['point_step'])
                    print(f"  Using point_step={point_step} from metadata")
                except:
                    pass
        
        # Try different approaches to parse the point cloud data
        
        # Approach 1: Try to find point cloud data after the header
        # ROS PointCloud2 messages typically have a header followed by point data
        header_size_guesses = [100, 200, 300, 400, 500]
        
        for header_size in header_size_guesses:
            if header_size >= len(data):
                continue
                
            point_data = data[header_size:]
            
            # Calculate how many points we can extract
            num_points = len(point_data) // point_step
            
            if num_points > 0:
                try:
                    # Create an empty array for the points
                    points = np.zeros((num_points, 3), dtype=np.float32)
                    
                    # Extract XYZ coordinates from each point
                    for i in range(num_points):
                        offset = i * point_step
                        # Assuming the first 12 bytes (3 floats) are XYZ
                        if offset + 12 <= len(point_data):
                            x = struct.unpack_from('f', point_data, offset)[0]
                            y = struct.unpack_from('f', point_data, offset + 4)[0]
                            z = struct.unpack_from('f', point_data, offset + 8)[0]
                            points[i] = [x, y, z]
                    
                    # Save as numpy array
                    np.save(output_file, points)
                    print(f"  Successfully converted to point cloud with {num_points} points")
                    
                    # Also save a JSON with metadata
                    json_file = os.path.join(output_dir, f"{name_without_ext}_meta.json")
                    with open(json_file, 'w') as f:
                        json.dump({
                            'num_points': int(num_points),
                            'point_step': point_step,
                            'header_size': header_size
                        }, f, indent=2)
                    
                    return output_file
                except Exception as e:
                    print(f"  Failed conversion with header_size={header_size}: {e}")
        
        # If we get here, all approaches failed
        print(f"  Failed to convert {raw_file_path} to point cloud")
        return None
    
    except Exception as e:
        print(f"  Error converting {raw_file_path}: {e}")
        return None

def process_directory(directory_path):
    """
    Process all raw files in a directory.
    
    Args:
        directory_path: Path to the directory containing raw files
    """
    print(f"Processing directory: {directory_path}")
    
    # Create a 'converted' subdirectory
    converted_dir = os.path.join(directory_path, "converted")
    os.makedirs(converted_dir, exist_ok=True)
    
    # Find all raw binary files
    raw_files = [f for f in os.listdir(directory_path) if f.endswith('.bin')]
    
    if not raw_files:
        print(f"  No raw binary files found in {directory_path}")
        return
    
    print(f"  Found {len(raw_files)} raw binary files")
    
    # Determine if this is a camera or LiDAR directory
    is_camera = any(keyword in os.path.basename(directory_path).lower() for keyword in ['rgb', 'camera', 'image'])
    is_lidar = any(keyword in os.path.basename(directory_path).lower() for keyword in ['lidar', 'point', 'cloud'])
    
    # Process each raw file
    for raw_file in raw_files:
        raw_path = os.path.join(directory_path, raw_file)
        
        if is_camera:
            convert_raw_camera_image(raw_path, converted_dir)
        elif is_lidar:
            convert_raw_lidar_data(raw_path, converted_dir)
        else:
            print(f"  Skipping {raw_file} - unknown data type")

def main():
    # Get the extracted data directory
    extracted_data_dir = "extracted_data"
    
    if not os.path.exists(extracted_data_dir):
        print(f"Error: Directory {extracted_data_dir} not found")
        return
    
    # Process each subdirectory
    for subdir in os.listdir(extracted_data_dir):
        subdir_path = os.path.join(extracted_data_dir, subdir)
        
        if os.path.isdir(subdir_path):
            process_directory(subdir_path)

if __name__ == "__main__":
    main()
