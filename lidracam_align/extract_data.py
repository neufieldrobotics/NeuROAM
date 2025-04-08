#!/usr/bin/env python3
"""
Script to extract and visualize data from a ROS bag file containing LiDAR and camera data.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
import cv2
from datetime import datetime
import struct

def extract_data(bag_path, output_dir="extracted_data"):
    """Extract camera and LiDAR data from a ROS bag file."""
    print(f"Extracting data from bag file: {bag_path}")
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Check if the file exists
    if not os.path.exists(bag_path):
        print(f"Error: Bag file not found at {bag_path}")
        return
    
    # Open the bag file
    with Reader(bag_path) as reader:
        # Print bag info
        print(f"Bag duration: {reader.duration:.2f} seconds")
        print(f"Start time: {datetime.fromtimestamp(reader.start_time/1e9)}")
        print(f"End time: {datetime.fromtimestamp(reader.end_time/1e9)}")
        print(f"Message count: {reader.message_count}")
        
        # Get typestore for ROS1 and ROS2
        typestore_ros1 = get_typestore(Stores.ROS1_NOETIC)
        typestore_ros2 = get_typestore(Stores.ROS2_GALACTIC)
        
        # Find camera and LiDAR connections
        camera_connections = []
        lidar_connections = []
        
        for connection in reader.connections:
            if 'sensor_msgs/msg/Image' in connection.msgtype or 'sensor_msgs/Image' in connection.msgtype:
                camera_connections.append(connection)
                print(f"Found camera topic: {connection.topic}")
            elif 'sensor_msgs/msg/PointCloud2' in connection.msgtype or 'sensor_msgs/PointCloud2' in connection.msgtype:
                lidar_connections.append(connection)
                print(f"Found LiDAR topic: {connection.topic}")
        
        # Extract camera images
        if camera_connections:
            print("\nExtracting camera images...")
            
            # Create subdirectories for each camera
            for connection in camera_connections:
                camera_dir = os.path.join(output_dir, connection.topic.replace('/', '_').lstrip('_'))
                os.makedirs(camera_dir, exist_ok=True)
                
                print(f"Processing camera: {connection.topic}")
                msg_count = 0
                
                # Determine if it's a ROS1 or ROS2 message
                is_ros2 = 'msg' in connection.msgtype
                typestore = typestore_ros2 if is_ros2 else typestore_ros1
                
                # Iterate through messages for this connection
                for connection, timestamp, data in reader.messages([connection]):
                    try:
                        # Skip the first few bytes which might be causing issues
                        if len(data) < 10:  # Skip if data is too small
                            continue
                            
                        # Try to deserialize with appropriate typestore
                        try:
                            msg = typestore.deserialize_cdr(data, connection.msgtype)
                        except Exception as e:
                            print(f"  Error deserializing with CDR: {e}")
                            # Try alternative approach - just save the raw data
                            raw_filename = os.path.join(camera_dir, f"raw_frame_{msg_count:06d}.bin")
                            with open(raw_filename, 'wb') as f:
                                f.write(data)
                            msg_count += 1
                            if msg_count >= 10:  # Limit to 10 raw frames per camera
                                break
                            continue
                        
                        # Get encoding
                        encoding = getattr(msg, 'encoding', 'rgb8')
                        
                        # Extract image data
                        if hasattr(msg, 'data'):
                            img_data = np.frombuffer(msg.data, dtype=np.uint8)
                            if hasattr(msg, 'height') and hasattr(msg, 'width') and hasattr(msg, 'step'):
                                try:
                                    # Reshape based on dimensions
                                    if len(img_data) == msg.height * msg.width:
                                        # Grayscale image
                                        img = img_data.reshape(msg.height, msg.width)
                                    else:
                                        # Color image
                                        channels = 3  # Default to 3 channels (RGB)
                                        if 'bgra' in encoding.lower() or 'rgba' in encoding.lower():
                                            channels = 4
                                        elif 'mono' in encoding.lower() or 'gray' in encoding.lower():
                                            channels = 1
                                        
                                        if channels == 1:
                                            img = img_data.reshape(msg.height, msg.width)
                                        else:
                                            # Make sure the data size matches the expected size
                                            expected_size = msg.height * msg.width * channels
                                            if len(img_data) >= expected_size:
                                                img = img_data[:expected_size].reshape(msg.height, msg.width, channels)
                                            else:
                                                print(f"  Data size mismatch: expected {expected_size}, got {len(img_data)}")
                                                continue
                                    
                                    # Save the image
                                    filename = os.path.join(camera_dir, f"frame_{msg_count:06d}.png")
                                    cv2.imwrite(filename, img)
                                    
                                    # Only extract a limited number of frames
                                    msg_count += 1
                                    if msg_count % 10 == 0:
                                        print(f"  Extracted {msg_count} frames...")
                                    
                                    if msg_count >= 50:  # Limit to 50 frames per camera
                                        break
                                        
                                except Exception as e:
                                    print(f"  Error reshaping image: {e}")
                            else:
                                print(f"  Missing image dimensions")
                        else:
                            print(f"  No image data found in message")
                    except Exception as e:
                        print(f"  Error processing image: {e}")
                
                print(f"  Total frames extracted: {msg_count}")
        
        # Extract LiDAR point clouds
        if lidar_connections:
            print("\nExtracting LiDAR point clouds...")
            
            # Create subdirectory for LiDAR data
            for connection in lidar_connections:
                lidar_dir = os.path.join(output_dir, connection.topic.replace('/', '_').lstrip('_'))
                os.makedirs(lidar_dir, exist_ok=True)
                
                print(f"Processing LiDAR: {connection.topic}")
                msg_count = 0
                
                # Determine if it's a ROS1 or ROS2 message
                is_ros2 = 'msg' in connection.msgtype
                typestore = typestore_ros2 if is_ros2 else typestore_ros1
                
                # Iterate through messages for this connection
                for connection, timestamp, data in reader.messages([connection]):
                    try:
                        # Try to deserialize with appropriate typestore
                        try:
                            msg = typestore.deserialize_cdr(data, connection.msgtype)
                        except Exception as e:
                            print(f"  Error deserializing point cloud: {e}")
                            # Save raw data
                            raw_filename = os.path.join(lidar_dir, f"raw_cloud_{msg_count:06d}.bin")
                            with open(raw_filename, 'wb') as f:
                                f.write(data)
                            msg_count += 1
                            if msg_count >= 5:  # Limit to 5 raw point clouds
                                break
                            continue
                        
                        # Save point cloud metadata
                        if msg_count == 0:
                            if hasattr(msg, 'fields'):
                                field_names = [field.name for field in msg.fields]
                                field_info = {
                                    'names': field_names,
                                    'point_step': getattr(msg, 'point_step', 0),
                                    'row_step': getattr(msg, 'row_step', 0),
                                    'is_dense': getattr(msg, 'is_dense', False),
                                    'width': getattr(msg, 'width', 0),
                                    'height': getattr(msg, 'height', 0)
                                }
                                print(f"  Point cloud fields: {field_names}")
                                print(f"  Point step: {field_info['point_step']} bytes")
                                print(f"  Is dense: {field_info['is_dense']}")
                                
                                # Save metadata
                                metadata_file = os.path.join(lidar_dir, "metadata.txt")
                                with open(metadata_file, 'w') as f:
                                    for key, value in field_info.items():
                                        f.write(f"{key}: {value}\n")
                        
                        # Save raw point cloud data
                        if hasattr(msg, 'data'):
                            # Save binary data
                            filename = os.path.join(lidar_dir, f"cloud_{msg_count:06d}.bin")
                            with open(filename, 'wb') as f:
                                f.write(msg.data)
                            
                            # Only extract a limited number of point clouds
                            msg_count += 1
                            if msg_count % 5 == 0:
                                print(f"  Extracted {msg_count} point clouds...")
                            
                            if msg_count >= 20:  # Limit to 20 point clouds
                                break
                        else:
                            print(f"  No point cloud data found in message")
                    except Exception as e:
                        print(f"  Error processing point cloud: {e}")
                
                print(f"  Total point clouds extracted: {msg_count}")

def visualize_sample_data(data_dir="extracted_data"):
    """Visualize sample extracted data."""
    print(f"Visualizing sample data from {data_dir}")
    
    # Check if the directory exists
    if not os.path.exists(data_dir):
        print(f"Error: Data directory not found at {data_dir}")
        return
    
    # Find subdirectories (one per topic)
    subdirs = [d for d in os.listdir(data_dir) if os.path.isdir(os.path.join(data_dir, d))]
    
    # Visualize camera images
    camera_dirs = [d for d in subdirs if 'rgb' in d or 'image' in d or 'camera' in d]
    
    if camera_dirs:
        print("\nVisualizing camera images:")
        
        for camera_dir in camera_dirs:
            print(f"Camera: {camera_dir}")
            camera_path = os.path.join(data_dir, camera_dir)
            
            # Find image files
            image_files = [f for f in os.listdir(camera_path) if f.endswith('.png')]
            image_files.sort()
            
            if image_files:
                # Display a sample image
                sample_image = os.path.join(camera_path, image_files[0])
                img = cv2.imread(sample_image)
                
                if img is not None:
                    # Convert BGR to RGB for matplotlib
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    
                    plt.figure(figsize=(10, 6))
                    plt.imshow(img_rgb)
                    plt.title(f"Sample image from {camera_dir}")
                    plt.axis('off')
                    
                    # Save the figure
                    plt.savefig(os.path.join(data_dir, f"{camera_dir}_sample.png"))
                    plt.close()
                    
                    print(f"  Saved sample image visualization to {os.path.join(data_dir, f'{camera_dir}_sample.png')}")
                else:
                    print(f"  Error reading image: {sample_image}")
            else:
                print(f"  No image files found in {camera_path}")
    else:
        print("No camera directories found")

if __name__ == "__main__":
    bag_path = "lidar_camera_ros_data/carla-v2.2-t10.bag"
    extract_data(bag_path)
    visualize_sample_data()
