#!/usr/bin/env python3

import argparse
import os
import sys
import shutil
import yaml
import numpy as np
import cv2
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Set
import tempfile

# ROS2 imports
try:
    import rclpy
    from rclpy.serialization import deserialize_message, serialize_message
    from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
    from rosbag2_py import TopicMetadata
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    from cv_bridge import CvBridge
except ImportError as e:
    print(f"Error importing ROS2 packages: {e}")
    print("Make sure you have sourced ROS2 and installed required packages:")
    print("  source /opt/ros/humble/setup.bash")
    print("  pip install rosbag2-py")
    sys.exit(1)

# ROS1 imports (optional)
try:
    import rosbag as rosbag1
    import rospy
    from sensor_msgs.msg import Image as Image1, CompressedImage as CompressedImage1
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False
    print("Warning: ROS1 packages not available. ROS1 conversion will be disabled.")
    print("To enable ROS1 conversion, install: pip install rosbag rospkg")


class BagProcessor:
    def __init__(self, input_bag: str, output_bag: Optional[str] = None, 
                 in_place: bool = False, force_overwrite: bool = False,
                 config_file: Optional[str] = None, enable_debayer: bool = False):
        """
        Initialize the bag processor.
        
        Args:
            input_bag: Path to input ROS2 bag folder
            output_bag: Path to output ROS2 bag folder (optional)
            in_place: Whether to modify the bag in place
            force_overwrite: Whether to overwrite existing output
            config_file: Path to sensor topics configuration YAML file
            enable_debayer: Whether to enable camera debayering
        """
        self.input_bag = Path(input_bag)
        self.in_place = in_place
        self.force_overwrite = force_overwrite
        self.enable_debayer = enable_debayer
        
        if not self.input_bag.exists():
            raise FileNotFoundError(f"Input bag folder not found: {input_bag}")
        
        # Check for metadata.yaml
        metadata_path = self.input_bag / "metadata.yaml"
        if not metadata_path.exists():
            raise FileNotFoundError(f"metadata.yaml not found in {input_bag}")
        
        # Set output bag path
        if in_place:
            self.output_bag = self.input_bag
            self.temp_bag = Path(tempfile.mkdtemp(prefix="ros2bag_temp_"))
        elif output_bag:
            self.output_bag = Path(output_bag)
        else:
            # Create output bag name with suffix
            suffix = "_processed" if enable_debayer else "_filtered"
            self.output_bag = self.input_bag.parent / f"{self.input_bag.name}{suffix}"
        
        # Handle existing output directory
        if not in_place and self.output_bag.exists():
            if force_overwrite:
                print(f"Warning: Output directory exists and will be overwritten: {self.output_bag}")
            else:
                # Create unique output name
                counter = 1
                base_name = str(self.output_bag)
                while self.output_bag.exists():
                    self.output_bag = Path(f"{base_name}_{counter}")
                    counter += 1
                print(f"Output directory already exists, using: {self.output_bag}")
        
        self.bridge = CvBridge()
        
        # Default topic names for camera processing
        self.cam0_compressed_topic = "/cam_sync/cam0/image_raw/compressed"
        self.cam1_compressed_topic = "/cam_sync/cam1/image_raw/compressed"
        self.cam0_debayered_topic = "/cam_sync/cam0/image_raw/debayered"
        self.cam1_debayered_topic = "/cam_sync/cam1/image_raw/debayered"
        
        # Topics to check for already debayered images
        self.existing_debayered_topics = {
            self.cam0_debayered_topic,
            self.cam1_debayered_topic,
            "/cam_sync/cam0/image_raw",
            "/cam_sync/cam1/image_raw"
        }
        
        # Storage format detection
        self.storage_id = self._detect_storage_format()
        
        # Load sensor configuration
        self.config = None
        self.selected_groups = []
        self.selected_topics = None  # Will be set if filtering is enabled
        self.exclude_patterns = []
        self.replace_compressed_with_debayered = False  # Will be set based on camera group selection
        
        if config_file and Path(config_file).exists():
            self.load_config(config_file)
    
    def _detect_storage_format(self) -> str:
        """Detect the storage format from metadata.yaml"""
        metadata_path = self.input_bag / "metadata.yaml"
        with open(metadata_path, 'r') as f:
            metadata = yaml.safe_load(f)
        
        # Check for storage format in metadata
        storage_id = metadata.get('rosbag2_bagfile_information', {}).get('storage_identifier', 'mcap')
        return storage_id
    
    def load_config(self, config_file: str):
        """Load sensor topics configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"Loaded sensor configuration from: {config_file}")
            
            # Load exclude patterns
            if 'exclude_patterns' in self.config:
                self.exclude_patterns = [re.compile(p) for p in self.config['exclude_patterns']]
                
        except Exception as e:
            print(f"Warning: Failed to load config file {config_file}: {e}")
            self.config = None
    
    def set_camera_topics(self, cam0_topic: str, cam1_topic: str):
        """Set custom camera topic names"""
        self.cam0_compressed_topic = cam0_topic
        self.cam1_compressed_topic = cam1_topic
        # Update debayered topic names based on compressed topic names
        self.cam0_debayered_topic = cam0_topic.replace('/compressed', '/debayered')
        self.cam1_debayered_topic = cam1_topic.replace('/compressed', '/debayered')
        
    def set_sensor_groups(self, groups: List[str], preset: Optional[str] = None):
        """
        Set which sensor groups to include in the output bag
        
        Args:
            groups: List of sensor groups ('camera', 'lidar', 'imu', 'gps')
            preset: Optional preset name from config file
        """
        if not self.config:
            print("Warning: No configuration loaded, sensor filtering disabled")
            return
        
        self.selected_groups = groups
        self.selected_topics = set()
        
        # Check if camera group is selected and debayering is enabled
        if 'camera' in groups and self.enable_debayer:
            self.replace_compressed_with_debayered = True
            print("Camera group selected with debayering: will replace compressed topics with debayered")
        
        # Handle preset if specified
        if preset and 'presets' in self.config and preset in self.config['presets']:
            preset_config = self.config['presets'][preset]
            print(f"Using preset '{preset}': {preset_config['description']}")
            
            # Add groups from preset
            for group in preset_config['groups']:
                if group not in groups:
                    groups.append(group)
                    
            # Add additional topics from preset
            if 'additional_topics' in preset_config:
                self.selected_topics.update(preset_config['additional_topics'])
        
        # Add topics from selected groups
        for group in groups:
            if group in self.config:
                topics = self.config[group].get('topics', [])
                
                # If camera group and debayering enabled, modify topic list
                if group == 'camera' and self.replace_compressed_with_debayered:
                    modified_topics = []
                    for topic in topics:
                        # Skip compressed topics as they'll be replaced by debayered
                        if 'compressed' in topic:
                            continue
                        # Keep camera_info and debayered topics
                        modified_topics.append(topic)
                    # Ensure debayered topics are included
                    modified_topics.append(self.cam0_debayered_topic)
                    modified_topics.append(self.cam1_debayered_topic)
                    self.selected_topics.update(modified_topics)
                    print(f"Including camera topics with debayering ({len(modified_topics)} topics)")
                else:
                    self.selected_topics.update(topics)
                    print(f"Including {group} topics ({len(topics)} topics)")
        
        # Always include system topics
        if 'always_include' in self.config:
            always_topics = self.config['always_include'].get('topics', [])
            self.selected_topics.update(always_topics)
            
        print(f"Total topics to include: {len(self.selected_topics)}")
        
    def should_include_topic(self, topic_name: str) -> bool:
        """
        Check if a topic should be included based on filtering rules
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            True if topic should be included, False otherwise
        """
        # If no filtering, include everything
        if self.selected_topics is None:
            return True
            
        # Check if topic matches any exclude pattern
        for pattern in self.exclude_patterns:
            if pattern.match(topic_name):
                return False
        
        # If replacing compressed with debayered, exclude compressed camera topics
        if self.replace_compressed_with_debayered:
            if topic_name in [self.cam0_compressed_topic, self.cam1_compressed_topic]:
                return False
                
        # Check if topic is in selected topics
        return topic_name in self.selected_topics
    
    def is_already_debayered(self, topic_name: str) -> bool:
        """Check if a topic is already debayered"""
        return topic_name in self.existing_debayered_topics or 'debayered' in topic_name
    
    def debayer_image(self, compressed_msg: CompressedImage, camera_id: int) -> Image:
        """
        Debayer a compressed image following the logic from debayer_node.py
        
        Args:
            compressed_msg: Compressed image message
            camera_id: 0 for cam0, 1 for cam1
            
        Returns:
            Debayered Image message
        """
        # Decode compressed image
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        bayer_img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        
        if bayer_img is None:
            print(f"Warning: Failed to decode image for camera {camera_id}")
            return None
        
        # Debayer based on camera ID
        if camera_id == 0:
            # Cam0 uses BAYER_RG2BGR
            rgb_img = cv2.cvtColor(bayer_img, cv2.COLOR_BAYER_RG2BGR)
        else:
            # Cam1 uses BAYER_RG2RGB
            rgb_img = cv2.cvtColor(bayer_img, cv2.COLOR_BAYER_RG2RGB)
        
        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="rgb8")
        
        # Preserve original header
        image_msg.header = compressed_msg.header
        
        return image_msg
    
    def process_bag(self):
        """Process the ROS2 bag to add debayered topics and/or filter topics"""
        print(f"Processing bag: {self.input_bag}")
        print(f"Output will be written to: {self.output_bag}")
        if self.enable_debayer:
            print("Debayering: ENABLED")
        else:
            print("Debayering: DISABLED")
        
        writer = None
        reader = None
        
        try:
            # Setup output path
            if self.in_place:
                output_path = str(self.temp_bag)
            else:
                output_path = str(self.output_bag)
                # Make sure output directory doesn't exist
                if Path(output_path).exists():
                    print(f"Removing existing output directory: {output_path}")
                    shutil.rmtree(output_path)
            
            # Setup reader
            reader = SequentialReader()
            storage_options = StorageOptions(
                uri=str(self.input_bag),
                storage_id=self.storage_id
            )
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            reader.open(storage_options, converter_options)
            
            # Setup writer
            writer = SequentialWriter()
            storage_options_out = StorageOptions(
                uri=output_path,
                storage_id=self.storage_id
            )
            writer.open(storage_options_out, converter_options)
            
            # Get all topics from the original bag
            topics_and_types = reader.get_all_topics_and_types()
            
            # Check if topics are already debayered
            existing_topics = {t.name for t in topics_and_types}
            has_debayered_cam0 = any(self.is_already_debayered(t) and 'cam0' in t for t in existing_topics)
            has_debayered_cam1 = any(self.is_already_debayered(t) and 'cam1' in t for t in existing_topics)
            
            if has_debayered_cam0 or has_debayered_cam1:
                print("Warning: Bag already contains debayered topics. Skipping debayering.")
                self.enable_debayer = False
            
            # Track which topics we're including
            included_topics = set()
            excluded_topics = set()
            
            # Determine if we should add debayered topics
            should_add_cam0_debayered = False
            should_add_cam1_debayered = False
            
            # Create topic metadata for filtered topics
            for topic_info in topics_and_types:
                # Skip compressed camera topics if we're replacing them with debayered
                if self.replace_compressed_with_debayered and self.enable_debayer:
                    if topic_info.name == self.cam0_compressed_topic:
                        should_add_cam0_debayered = True
                        excluded_topics.add(topic_info.name)
                        continue
                    elif topic_info.name == self.cam1_compressed_topic:
                        should_add_cam1_debayered = True
                        excluded_topics.add(topic_info.name)
                        continue
                
                if self.should_include_topic(topic_info.name):
                    topic_metadata = TopicMetadata(
                        name=topic_info.name,
                        type=topic_info.type,
                        serialization_format='cdr'
                    )
                    writer.create_topic(topic_metadata)
                    included_topics.add(topic_info.name)
                else:
                    excluded_topics.add(topic_info.name)
            
            # Add debayered topics if needed
            image_type = 'sensor_msgs/msg/Image'
            
            # Check if we should process cameras
            has_cam0 = self.cam0_compressed_topic in existing_topics
            has_cam1 = self.cam1_compressed_topic in existing_topics
            
            # Add debayered topics to writer if needed
            if should_add_cam0_debayered and has_cam0 and self.enable_debayer:
                cam0_debayered_meta = TopicMetadata(
                    name=self.cam0_debayered_topic,
                    type=image_type,
                    serialization_format='cdr'
                )
                writer.create_topic(cam0_debayered_meta)
                included_topics.add(self.cam0_debayered_topic)
                print(f"Will create debayered topic: {self.cam0_debayered_topic}")
            
            if should_add_cam1_debayered and has_cam1 and self.enable_debayer:
                cam1_debayered_meta = TopicMetadata(
                    name=self.cam1_debayered_topic,
                    type=image_type,
                    serialization_format='cdr'
                )
                writer.create_topic(cam1_debayered_meta)
                included_topics.add(self.cam1_debayered_topic)
                print(f"Will create debayered topic: {self.cam1_debayered_topic}")
            
            # If not replacing but still debayering (all topics mode with debayer flag)
            if self.enable_debayer and not self.replace_compressed_with_debayered:
                if has_cam0 and not has_debayered_cam0 and self.should_include_topic(self.cam0_debayered_topic):
                    cam0_debayered_meta = TopicMetadata(
                        name=self.cam0_debayered_topic,
                        type=image_type,
                        serialization_format='cdr'
                    )
                    writer.create_topic(cam0_debayered_meta)
                    included_topics.add(self.cam0_debayered_topic)
                    print(f"Added debayered topic: {self.cam0_debayered_topic}")
                    should_add_cam0_debayered = True
                
                if has_cam1 and not has_debayered_cam1 and self.should_include_topic(self.cam1_debayered_topic):
                    cam1_debayered_meta = TopicMetadata(
                        name=self.cam1_debayered_topic,
                        type=image_type,
                        serialization_format='cdr'
                    )
                    writer.create_topic(cam1_debayered_meta)
                    included_topics.add(self.cam1_debayered_topic)
                    print(f"Added debayered topic: {self.cam1_debayered_topic}")
                    should_add_cam1_debayered = True
            
            # Report filtering results
            if self.selected_topics is not None:
                print(f"\nTopic filtering enabled:")
                print(f"  - Including {len(included_topics)} topics")
                print(f"  - Excluding {len(excluded_topics)} topics")
                if len(excluded_topics) > 0 and len(excluded_topics) <= 10:
                    print(f"  - Excluded topics: {', '.join(sorted(excluded_topics))}")
                elif len(excluded_topics) > 10:
                    print(f"  - Excluded topics: {', '.join(sorted(list(excluded_topics)[:10]))}... and {len(excluded_topics)-10} more")
            
            # Process messages
            message_count = 0
            written_count = 0
            skipped_count = 0
            debayered_count = 0
            
            print("\nProcessing messages...")
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                
                # Check if we should skip compressed topics (when replacing with debayered)
                if self.replace_compressed_with_debayered and self.enable_debayer:
                    if topic in [self.cam0_compressed_topic, self.cam1_compressed_topic]:
                        # Process and write debayered version instead
                        if topic == self.cam0_compressed_topic and should_add_cam0_debayered:
                            try:
                                compressed_msg = deserialize_message(data, CompressedImage)
                                debayered_msg = self.debayer_image(compressed_msg, 0)
                                if debayered_msg:
                                    serialized = serialize_message(debayered_msg)
                                    writer.write(self.cam0_debayered_topic, serialized, timestamp)
                                    debayered_count += 1
                                    written_count += 1
                            except Exception as e:
                                print(f"Warning: Failed to debayer cam0 image: {e}")
                        elif topic == self.cam1_compressed_topic and should_add_cam1_debayered:
                            try:
                                compressed_msg = deserialize_message(data, CompressedImage)
                                debayered_msg = self.debayer_image(compressed_msg, 1)
                                if debayered_msg:
                                    serialized = serialize_message(debayered_msg)
                                    writer.write(self.cam1_debayered_topic, serialized, timestamp)
                                    debayered_count += 1
                                    written_count += 1
                            except Exception as e:
                                print(f"Warning: Failed to debayer cam1 image: {e}")
                        skipped_count += 1
                        message_count += 1
                        continue
                
                # Skip if topic is filtered out
                if not self.should_include_topic(topic):
                    skipped_count += 1
                    message_count += 1
                    continue
                
                # Write original message
                writer.write(topic, data, timestamp)
                written_count += 1
                
                # Additionally process compressed camera images if debayering is enabled
                # (only when not replacing, i.e., keeping both compressed and debayered)
                if self.enable_debayer and not self.replace_compressed_with_debayered:
                    if topic == self.cam0_compressed_topic and should_add_cam0_debayered:
                        try:
                            compressed_msg = deserialize_message(data, CompressedImage)
                            debayered_msg = self.debayer_image(compressed_msg, 0)
                            if debayered_msg:
                                serialized = serialize_message(debayered_msg)
                                writer.write(self.cam0_debayered_topic, serialized, timestamp)
                                debayered_count += 1
                        except Exception as e:
                            print(f"Warning: Failed to debayer cam0 image: {e}")
                            
                    elif topic == self.cam1_compressed_topic and should_add_cam1_debayered:
                        try:
                            compressed_msg = deserialize_message(data, CompressedImage)
                            debayered_msg = self.debayer_image(compressed_msg, 1)
                            if debayered_msg:
                                serialized = serialize_message(debayered_msg)
                                writer.write(self.cam1_debayered_topic, serialized, timestamp)
                                debayered_count += 1
                        except Exception as e:
                            print(f"Warning: Failed to debayer cam1 image: {e}")
                
                message_count += 1
                if message_count % 1000 == 0:
                    status = f"Processed {message_count} messages"
                    if self.selected_topics is not None:
                        status += f" (written: {written_count}, skipped: {skipped_count})"
                    if self.enable_debayer:
                        status += f", debayered: {debayered_count}"
                    print(status + "...")
            
            print(f"\nProcessing complete!")
            print(f"Total messages processed: {message_count}")
            if self.selected_topics is not None:
                print(f"  - Written to output: {written_count}")
                print(f"  - Filtered out: {skipped_count}")
            if self.enable_debayer:
                print(f"  - Debayered images created: {debayered_count}")
            
        except Exception as e:
            print(f"Error during processing: {e}")
            # Clean up partial output if not in-place
            if not self.in_place and self.output_bag.exists():
                print(f"Cleaning up partial output: {self.output_bag}")
                try:
                    shutil.rmtree(self.output_bag)
                except:
                    pass
            raise
            
        finally:
            # Clean up reader and writer objects
            if reader is not None:
                del reader
            if writer is not None:
                del writer
        
        # If in-place, move temp bag to original location
        if self.in_place:
            print(f"Moving processed bag to original location...")
            shutil.rmtree(self.input_bag)
            shutil.move(self.temp_bag, self.input_bag)
            print(f"In-place modification complete")
    
    def convert_to_ros1(self, output_path: Optional[str] = None):
        """Convert the processed ROS2 bag to ROS1 format"""
        if not ROS1_AVAILABLE:
            print("Error: ROS1 packages not available. Cannot convert to ROS1.")
            print("Install with: pip install rosbag rospkg")
            return False
        
        if output_path is None:
            output_path = str(self.output_bag) + ".bag"
        
        print(f"Converting to ROS1 bag: {output_path}")
        
        # Open ROS2 bag for reading
        reader = SequentialReader()
        storage_options = StorageOptions(
            uri=str(self.output_bag),
            storage_id=self.storage_id
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)
        
        # Create ROS1 bag
        ros1_bag = rosbag1.Bag(output_path, 'w')
        
        # Message type mapping
        type_map = {
            'sensor_msgs/msg/Image': Image1,
            'sensor_msgs/msg/CompressedImage': CompressedImage1,
            # Add more mappings as needed for your specific sensors
        }
        
        message_count = 0
        skipped_count = 0
        
        try:
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                
                # Get topic type
                topic_type = None
                for topic_info in reader.get_all_topics_and_types():
                    if topic_info.name == topic:
                        topic_type = topic_info.type
                        break
                
                if topic_type in type_map:
                    # Deserialize ROS2 message
                    if topic_type == 'sensor_msgs/msg/Image':
                        ros2_msg = deserialize_message(data, Image)
                        # Convert to ROS1 message
                        ros1_msg = Image1()
                        ros1_msg.header.seq = 0  # ROS1 specific
                        ros1_msg.header.stamp = rospy.Time(
                            secs=ros2_msg.header.stamp.sec,
                            nsecs=ros2_msg.header.stamp.nanosec
                        )
                        ros1_msg.header.frame_id = ros2_msg.header.frame_id
                        ros1_msg.height = ros2_msg.height
                        ros1_msg.width = ros2_msg.width
                        ros1_msg.encoding = ros2_msg.encoding
                        ros1_msg.is_bigendian = ros2_msg.is_bigendian
                        ros1_msg.step = ros2_msg.step
                        ros1_msg.data = ros2_msg.data
                        
                    elif topic_type == 'sensor_msgs/msg/CompressedImage':
                        ros2_msg = deserialize_message(data, CompressedImage)
                        ros1_msg = CompressedImage1()
                        ros1_msg.header.seq = 0
                        ros1_msg.header.stamp = rospy.Time(
                            secs=ros2_msg.header.stamp.sec,
                            nsecs=ros2_msg.header.stamp.nanosec
                        )
                        ros1_msg.header.frame_id = ros2_msg.header.frame_id
                        ros1_msg.format = ros2_msg.format
                        ros1_msg.data = ros2_msg.data
                    
                    # Write to ROS1 bag
                    ros1_time = rospy.Time(nsecs=timestamp)
                    ros1_bag.write(topic, ros1_msg, ros1_time)
                    message_count += 1
                else:
                    skipped_count += 1
                    if skipped_count <= 10:  # Only print first 10 warnings
                        print(f"Warning: Skipping unsupported message type: {topic_type} on topic {topic}")
                
                if message_count % 1000 == 0:
                    print(f"Converted {message_count} messages...")
                    
        finally:
            if 'ros1_bag' in locals():
                ros1_bag.close()
            if 'reader' in locals():
                del reader
        
        print(f"ROS1 conversion complete!")
        print(f"Messages converted: {message_count}")
        print(f"Messages skipped: {skipped_count}")
        print(f"ROS1 bag saved to: {output_path}")
        
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Process ROS2 MCAP bags with debayering, filtering, and ROS1 conversion',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process bag with all topics (no debayering by default)
  %(prog)s /path/to/bag
  
  # Enable debayering for cameras
  %(prog)s /path/to/bag --debayer
  
  # Filter for LiDAR-based SLAM (LiDAR + IMU only)
  %(prog)s /path/to/bag --lidar --imu
  
  # Camera with debayering (replaces compressed with debayered)
  %(prog)s /path/to/bag --camera --debayer
  
  # Use a preset configuration
  %(prog)s /path/to/bag --preset lio_sam
  
  # Convert any bag to ROS1 without processing
  %(prog)s /path/to/bag --convert-ros1
  
  # Custom configuration file
  %(prog)s /path/to/bag --config my_sensors.yaml --camera --gps --debayer
        """
    )
    parser.add_argument('input_bag', help='Path to input ROS2 bag folder')
    parser.add_argument('--output-bag', help='Output bag folder path (default: input_bag_processed or _filtered)')
    parser.add_argument('--in-place', action='store_true', 
                       help='Modify the original bag in place')
    parser.add_argument('--force', '-f', action='store_true',
                       help='Force overwrite existing output bag')
    parser.add_argument('--convert-ros1', action='store_true',
                       help='Convert the bag to ROS1 format after processing')
    parser.add_argument('--ros1-output', help='ROS1 bag output path (used with --convert-ros1)')
    
    # Debayering control
    parser.add_argument('--debayer', action='store_true',
                       help='Enable camera debayering (off by default)')
    
    # Sensor configuration arguments
    parser.add_argument('--config', default=None,
                       help='Path to sensor topics configuration YAML file (default: sensor_topics.yaml in script dir)')
    parser.add_argument('--camera', action='store_true',
                       help='Include camera topics (use with --debayer to replace compressed with debayered)')
    parser.add_argument('--lidar', action='store_true',
                       help='Include LiDAR topics')
    parser.add_argument('--imu', action='store_true',
                       help='Include IMU topics')
    parser.add_argument('--gps', action='store_true',
                       help='Include GPS topics')
    parser.add_argument('--preset', choices=['lio_sam', 'vins_fusion', 'full_slam'],
                       help='Use a preset configuration for common SLAM systems')
    parser.add_argument('--all-topics', action='store_true',
                       help='Include all topics (default if no sensor groups specified)')
    
    # Camera topic customization
    parser.add_argument('--cam0-topic', 
                       default='/cam_sync/cam0/image_raw/compressed',
                       help='Cam0 compressed topic name')
    parser.add_argument('--cam1-topic', 
                       default='/cam_sync/cam1/image_raw/compressed',
                       help='Cam1 compressed topic name')
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.in_place and args.output_bag:
        print("Error: Cannot use both --in-place and --output-bag")
        sys.exit(1)
    
    # Determine config file path
    config_file = args.config
    if not config_file:
        # Look for default config file in script directory
        script_dir = Path(__file__).parent
        default_config = script_dir / "sensor_topics.yaml"
        if default_config.exists():
            config_file = str(default_config)
            print(f"Using default config file: {config_file}")
    
    # Determine which sensor groups to include
    sensor_groups = []
    if args.camera:
        sensor_groups.append('camera')
    if args.lidar:
        sensor_groups.append('lidar')
    if args.imu:
        sensor_groups.append('imu')
    if args.gps:
        sensor_groups.append('gps')
    
    # If no groups specified and not using all-topics, include all by default
    if not sensor_groups and not args.preset and not args.all_topics:
        print("Note: No sensor groups specified, including all topics")
        # Don't set sensor_groups, which means no filtering
    
    try:
        # Create processor
        processor = BagProcessor(
            input_bag=args.input_bag,
            output_bag=args.output_bag,
            in_place=args.in_place,
            force_overwrite=args.force,
            config_file=config_file,
            enable_debayer=args.debayer
        )
        
        # Set custom camera topics if provided
        processor.set_camera_topics(args.cam0_topic, args.cam1_topic)
        
        # Apply sensor group filtering if specified
        if sensor_groups or args.preset:
            if not processor.config:
                print("Warning: No configuration file loaded, cannot filter topics")
                print("Create a sensor_topics.yaml file or specify with --config")
            else:
                processor.set_sensor_groups(sensor_groups, args.preset)
        
        # Process the bag
        processor.process_bag()
        
        # Convert to ROS1 if requested
        if args.convert_ros1:
            if not ROS1_AVAILABLE:
                print("\nError: ROS1 conversion requested but ROS1 packages not available")
                print("Install with: pip install rosbag rospkg")
            else:
                processor.convert_to_ros1(args.ros1_output)
                
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()