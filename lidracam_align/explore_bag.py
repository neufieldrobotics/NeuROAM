#!/usr/bin/env python3
"""
Script to explore the contents of a ROS bag file and extract information about topics,
message types, and sample data.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr
import cv2
from datetime import datetime

def explore_bag(bag_path):
    """Explore the contents of a ROS bag file."""
    print(f"Exploring bag file: {bag_path}")
    
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
        
        # Print topic info
        print("\nTopics in the bag file:")
        for connection in reader.connections:
            topic = connection.topic
            msg_type = connection.msgtype
            print(f"  - {topic}: type: {msg_type}")
        
        # Count messages by type
        msg_types = {}
        for connection in reader.connections:
            if connection.msgtype not in msg_types:
                msg_types[connection.msgtype] = []
            if connection.topic not in msg_types[connection.msgtype]:
                msg_types[connection.msgtype].append(connection.topic)
        
        print("\nMessage types in the bag file:")
        for msg_type, topics in msg_types.items():
            print(f"  - {msg_type}: {len(topics)} topics")
            for topic in topics:
                print(f"    - {topic}")
        
        # Look for camera and lidar topics
        camera_topics = []
        lidar_topics = []
        
        for connection in reader.connections:
            if 'sensor_msgs/msg/Image' in connection.msgtype or 'sensor_msgs/Image' in connection.msgtype:
                if connection.topic not in camera_topics:
                    camera_topics.append(connection.topic)
            elif 'sensor_msgs/msg/PointCloud2' in connection.msgtype or 'sensor_msgs/PointCloud2' in connection.msgtype:
                if connection.topic not in lidar_topics:
                    lidar_topics.append(connection.topic)
        
        print("\nCamera topics:")
        for topic in camera_topics:
            print(f"  - {topic}")
        
        print("\nLiDAR topics:")
        for topic in lidar_topics:
            print(f"  - {topic}")
        
        # Try to extract and save a sample image from each camera topic
        if camera_topics:
            print("\nExtracting sample images from camera topics...")
            os.makedirs("sample_data", exist_ok=True)
            
            for topic in camera_topics:
                print(f"Processing topic: {topic}")
                # Get the connection for this topic
                connections = [c for c in reader.connections if c.topic == topic]
                if not connections:
                    print(f"  No connection found for topic {topic}")
                    continue
                
                connection = connections[0]
                try:
                    # Get a sample message
                    for timestamp, data in reader.messages(connections=[connection]):
                        try:
                            msg = deserialize_cdr(data, connection.msgtype)
                            
                            # Convert ROS image message to OpenCV image
                            if hasattr(msg, 'encoding'):
                                encoding = msg.encoding
                            else:
                                # Assume a common encoding if not specified
                                encoding = 'rgb8'
                            
                            if hasattr(msg, 'data'):
                                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                                if hasattr(msg, 'height') and hasattr(msg, 'width') and hasattr(msg, 'step'):
                                    img = img_data.reshape(msg.height, msg.width, -1)
                                    
                                    # Convert to BGR for OpenCV if needed
                                    if 'rgb' in encoding.lower():
                                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                                    
                                    # Save the image
                                    topic_name = topic.replace('/', '_').lstrip('_')
                                    filename = f"sample_data/{topic_name}_{timestamp}.png"
                                    cv2.imwrite(filename, img)
                                    print(f"  Saved image to {filename}")
                                    
                                    # Display basic image info
                                    print(f"  Image dimensions: {msg.width}x{msg.height}")
                                    print(f"  Encoding: {encoding}")
                                    break
                            else:
                                print(f"  No image data found in message")
                        except Exception as e:
                            print(f"  Error processing image: {e}")
                            continue
                        break  # Just process one message
                except Exception as e:
                    print(f"  Error accessing messages: {e}")
        
        # Try to extract information about a sample point cloud
        if lidar_topics:
            print("\nExtracting information about LiDAR point clouds...")
            
            for topic in lidar_topics:
                print(f"Processing topic: {topic}")
                # Get the connection for this topic
                connections = [c for c in reader.connections if c.topic == topic]
                if not connections:
                    print(f"  No connection found for topic {topic}")
                    continue
                
                connection = connections[0]
                try:
                    # Get a sample message
                    for timestamp, data in reader.messages(connections=[connection]):
                        try:
                            msg = deserialize_cdr(data, connection.msgtype)
                            
                            # Extract basic point cloud info
                            if hasattr(msg, 'width') and hasattr(msg, 'height'):
                                point_count = msg.width * msg.height
                                print(f"  Point cloud contains {point_count} points")
                                
                                if hasattr(msg, 'fields'):
                                    print(f"  Fields: {[field.name for field in msg.fields]}")
                                
                                if hasattr(msg, 'point_step') and hasattr(msg, 'row_step'):
                                    print(f"  Point step: {msg.point_step} bytes")
                                    print(f"  Row step: {msg.row_step} bytes")
                                
                                if hasattr(msg, 'is_dense'):
                                    print(f"  Is dense: {msg.is_dense}")
                                
                                # We don't parse the actual point cloud data here
                                # as it requires more complex processing
                            break
                        except Exception as e:
                            print(f"  Error processing point cloud: {e}")
                            continue
                        break  # Just process one message
                except Exception as e:
                    print(f"  Error accessing messages: {e}")

if __name__ == "__main__":
    bag_path = "lidar_camera_ros_data/carla-v2.2-t10.bag"
    explore_bag(bag_path)
