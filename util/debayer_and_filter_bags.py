#!/usr/bin/env python3
import argparse, os, sys, shutil, yaml, numpy as np, cv2, re, tempfile
from pathlib import Path
from typing import *

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from rosbag2_py import *
    from sensor_msgs.msg import Image, CompressedImage
    from cv_bridge import CvBridge
except ImportError as e:
    print(f"ROS2 import error: {e}\nSource ROS2 and install rosbag2-py")
    sys.exit(1)

try:
    import rosbag as rosbag1
    import rospy
    from sensor_msgs.msg import Image as Image1, CompressedImage as CompressedImage1
    ROS1_OK = True
except:
    ROS1_OK = False
    print("Warning: ROS1 packages not available. ROS1 conversion will be disabled.")
    print("To enable ROS1 conversion, install: pip install rosbag rospkg")


class BagProcessor:
    def __init__(self, input_bag, output_bag=None, in_place=False, force=False, config=None, debayer=False):
        self.input_bag = Path(input_bag)
        self.in_place = in_place
        self.force = force
        self.debayer = debayer
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
        if not self.input_bag.exists():
            raise FileNotFoundError(f"Bag not found: {input_bag}")
        if not (self.input_bag / "metadata.yaml").exists():
            raise FileNotFoundError(f"No metadata.yaml in {input_bag}")
        
        if in_place:
            self.output_bag = self.input_bag
            self.temp_bag = Path(tempfile.mkdtemp(prefix="ros2bag_"))
        elif output_bag:
            self.output_bag = Path(output_bag)
        else:
            self.output_bag = self.input_bag.parent / f"{self.input_bag.name}_{'processed' if debayer else 'filtered'}"
        
        if not in_place and self.output_bag.exists():
            if force:
                print(f"Will overwrite: {self.output_bag}")
            else:
                i = 1
                base = str(self.output_bag)
                while self.output_bag.exists():
                    self.output_bag = Path(f"{base}_{i}")
                    i += 1
                print(f"Output: {self.output_bag}")
        
        self.bridge = CvBridge()
        self.cam0_comp = "/cam_sync/cam0/image_raw/compressed"
        self.cam1_comp = "/cam_sync/cam1/image_raw/compressed"
        self.cam0_deb = "/cam_sync/cam0/image_raw/debayered"
        self.cam1_deb = "/cam_sync/cam1/image_raw/debayered"
        self.debayered_topics = {self.cam0_deb, self.cam1_deb, "/cam_sync/cam0/image_raw", "/cam_sync/cam1/image_raw"}
        
        with open(self.input_bag / "metadata.yaml") as f:
            self.storage_id = yaml.safe_load(f).get('rosbag2_bagfile_information', {}).get('storage_identifier', 'mcap')
        
        self.config = None
        self.selected_topics = None
        self.exclude_patterns = []
        self.replace_comp = False
        
        if config and Path(config).exists():
            with open(config) as f:
                self.config = yaml.safe_load(f)
                self.exclude_patterns = [re.compile(p) for p in self.config.get('exclude_patterns', [])]
    
    def set_camera_topics(self, cam0, cam1):
        self.cam0_comp = cam0
        self.cam1_comp = cam1
        self.cam0_deb = cam0.replace('/compressed', '/debayered')
        self.cam1_deb = cam1.replace('/compressed', '/debayered')
    
    def set_sensor_groups(self, groups):
        """
        Set which sensor groups to include in the output bag
        
        Args:
            groups: List of sensor groups ('camera', 'lidar', 'imu', 'gps')
        """
        if not self.config:
            print("Warning: No configuration loaded, sensor filtering disabled")
            return
        
        self.selected_topics = set()
        # Camera group automatically enables debayering and replacement
        if 'camera' in groups:
            self.debayer = True
            self.replace_comp = True
        self.replace_comp = 'camera' in groups and self.debayer
        
        for g in groups:
            if g in self.config:
                topics = self.config[g].get('topics', [])
                if g == 'camera' and self.replace_comp:
                    topics = [t for t in topics if 'compressed' not in t] + [self.cam0_deb, self.cam1_deb]
                self.selected_topics.update(topics)
        
        self.selected_topics.update(self.config.get('always_include', {}).get('topics', []))
    
    def should_include(self, topic):
        """
        Check if a topic should be included based on filtering rules
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            True if topic should be included, False otherwise
        """
        if self.selected_topics is None:
            return True
        if any(p.match(topic) for p in self.exclude_patterns):
            return False
        if self.replace_comp and topic in [self.cam0_comp, self.cam1_comp]:
            return False
        return topic in self.selected_topics
    
    def debayer_img(self, msg, cam_id):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None
        rgb = cv2.cvtColor(img, cv2.COLOR_BAYER_RG2BGR if cam_id == 0 else cv2.COLOR_BAYER_RG2RGB)
        out = self.bridge.cv2_to_imgmsg(rgb, "rgb8")
        out.header = msg.header
        return out
    
    def process_bag(self):
        print(f"Processing: {self.input_bag} -> {self.output_bag}")
        print(f"Debayer: {'ON' if self.debayer else 'OFF'}")
        
        out_path = str(self.temp_bag if self.in_place else self.output_bag)
        if Path(out_path).exists():
            shutil.rmtree(out_path)
        
        reader = SequentialReader()
        writer = SequentialWriter()
        opts = ConverterOptions('cdr', 'cdr')
        
        reader.open(StorageOptions(str(self.input_bag), self.storage_id), opts)
        writer.open(StorageOptions(out_path, self.storage_id), opts)
        
        topics = reader.get_all_topics_and_types()
        existing = {t.name for t in topics}
        
        # Check if already debayered
        if any('debayered' in t or t in self.debayered_topics for t in existing):
            print("Already debayered, skipping")
            self.debayer = False
        
        # Setup topics
        included = set()
        add_cam0 = add_cam1 = False
        
        for t in topics:
            if self.replace_comp and self.debayer:
                if t.name == self.cam0_comp:
                    add_cam0 = True
                    continue
                elif t.name == self.cam1_comp:
                    add_cam1 = True
                    continue
            
            if self.should_include(t.name):
                writer.create_topic(TopicMetadata(t.name, t.type, 'cdr'))
                included.add(t.name)
        
        # Add debayered topics
        if self.debayer:
            if add_cam0 and self.cam0_comp in existing:
                writer.create_topic(TopicMetadata(self.cam0_deb, 'sensor_msgs/msg/Image', 'cdr'))
                included.add(self.cam0_deb)
            if add_cam1 and self.cam1_comp in existing:
                writer.create_topic(TopicMetadata(self.cam1_deb, 'sensor_msgs/msg/Image', 'cdr'))
                included.add(self.cam1_deb)
            
            if not self.replace_comp:
                if self.cam0_comp in existing and self.cam0_deb not in existing:
                    writer.create_topic(TopicMetadata(self.cam0_deb, 'sensor_msgs/msg/Image', 'cdr'))
                    add_cam0 = True
                if self.cam1_comp in existing and self.cam1_deb not in existing:
                    writer.create_topic(TopicMetadata(self.cam1_deb, 'sensor_msgs/msg/Image', 'cdr'))
                    add_cam1 = True
        
        # Process messages
        count = written = deb_count = 0
        while reader.has_next():
            topic, data, ts = reader.read_next()
            
            # Replace compressed with debayered
            if self.replace_comp and self.debayer and topic in [self.cam0_comp, self.cam1_comp]:
                try:
                    msg = deserialize_message(data, CompressedImage)
                    deb = self.debayer_img(msg, 0 if topic == self.cam0_comp else 1)
                    if deb:
                        writer.write(self.cam0_deb if topic == self.cam0_comp else self.cam1_deb, 
                                   serialize_message(deb), ts)
                        deb_count += 1
                        written += 1
                except: pass
                count += 1
                continue
            
            if not self.should_include(topic):
                count += 1
                continue
            
            writer.write(topic, data, ts)
            written += 1
            
            # Add debayered alongside compressed
            if self.debayer and not self.replace_comp:
                if (topic == self.cam0_comp and add_cam0) or (topic == self.cam1_comp and add_cam1):
                    try:
                        msg = deserialize_message(data, CompressedImage)
                        deb = self.debayer_img(msg, 0 if topic == self.cam0_comp else 1)
                        if deb:
                            writer.write(self.cam0_deb if topic == self.cam0_comp else self.cam1_deb,
                                       serialize_message(deb), ts)
                            deb_count += 1
                    except: pass
            
            count += 1
            if count % 1000 == 0:
                print(f"Processed {count} msgs (written: {written}, debayered: {deb_count})")
        
        del reader, writer
        
        print(f"Done! Processed: {count}, Written: {written}, Debayered: {deb_count}")
        
        if self.in_place:
            shutil.rmtree(self.input_bag)
            shutil.move(self.temp_bag, self.input_bag)
    
    def convert_to_ros1(self, out_path=None):
        if not ROS1_OK:
            print("ROS1 not available")
            return False
        
        out_path = out_path or str(self.output_bag) + ".bag"
        print(f"Converting to ROS1: {out_path}")
        
        reader = SequentialReader()
        reader.open(StorageOptions(str(self.output_bag), self.storage_id), 
                   ConverterOptions('cdr', 'cdr'))
        
        bag1 = rosbag1.Bag(out_path, 'w')
        types = {'sensor_msgs/msg/Image': Image1, 'sensor_msgs/msg/CompressedImage': CompressedImage1}
        
        count = skip = 0
        while reader.has_next():
            topic, data, ts = reader.read_next()
            
            msg_type = next((t.type for t in reader.get_all_topics_and_types() if t.name == topic), None)
            
            if msg_type in types:
                if msg_type == 'sensor_msgs/msg/Image':
                    m2 = deserialize_message(data, Image)
                    m1 = Image1()
                    m1.header.seq = 0
                    m1.header.stamp = rospy.Time(m2.header.stamp.sec, m2.header.stamp.nanosec)
                    m1.header.frame_id = m2.header.frame_id
                    m1.height, m1.width, m1.encoding = m2.height, m2.width, m2.encoding
                    m1.is_bigendian, m1.step, m1.data = m2.is_bigendian, m2.step, m2.data
                else:
                    m2 = deserialize_message(data, CompressedImage)
                    m1 = CompressedImage1()
                    m1.header.seq = 0
                    m1.header.stamp = rospy.Time(m2.header.stamp.sec, m2.header.stamp.nanosec)
                    m1.header.frame_id = m2.header.frame_id
                    m1.format, m1.data = m2.format, m2.data
                
                bag1.write(topic, m1, rospy.Time(nsecs=ts))
                count += 1
            else:
                skip += 1
            
            if count % 1000 == 0:
                print(f"Converted {count} msgs")
        
        bag1.close()
        print(f"ROS1 done: {count} converted, {skip} skipped")
        return True

def main():
    p = argparse.ArgumentParser(description='Process ROS2 MCAP bags with debayering, filtering, and ROS1 conversion',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        Examples:
        # Process bag with all topics (no debayering by default)
        %(prog)s /path/to/bag
        
        # Enable debayering for cameras (will have both compressed and debayered camera topics)
        %(prog)s /path/to/bag --debayer
        
        # Filter for LiDAR-based SLAM (LiDAR + IMU only)
        %(prog)s /path/to/bag --lidar --imu

        # Camera with debayering (selecting camera enables debayering by default and replaces compressed topic with debayered)
        %(prog)s /path/to/bag --camera
        
        # Convert any bag to ROS1 without processing
        %(prog)s /path/to/bag --convert-ros1
        
        # Custom configuration file
        %(prog)s /path/to/bag --config my_sensors.yaml --camera --gps --debayer
                """
            )
    p.add_argument('input_bag')
    p.add_argument('--output-bag')
    p.add_argument('--in-place', action='store_true')
    p.add_argument('--force', '-f', action='store_true')
    p.add_argument('--convert-ros1', action='store_true')
    p.add_argument('--ros1-output')
    p.add_argument('--debayer', action='store_true')
    p.add_argument('--config')
    p.add_argument('--camera', action='store_true')
    p.add_argument('--lidar', action='store_true')
    p.add_argument('--imu', action='store_true')
    p.add_argument('--gps', action='store_true')
    p.add_argument('--cam0-topic', default='/cam_sync/cam0/image_raw/compressed')
    p.add_argument('--cam1-topic', default='/cam_sync/cam1/image_raw/compressed')
    
    args = p.parse_args()
    
    if args.in_place and args.output_bag:
        print("Can't use both --in-place and --output-bag")
        sys.exit(1)
    
    config_file = args.config
    if not config_file:
        default = Path(__file__).parent / "sensor_topics.yaml"
        if default.exists():
            config_file = str(default)
    
    groups = [g for g in ['camera', 'lidar', 'imu', 'gps'] if getattr(args, g)]
    
    proc = BagProcessor(args.input_bag, args.output_bag, args.in_place, 
                       args.force, config_file, args.debayer)
    proc.set_camera_topics(args.cam0_topic, args.cam1_topic)
    
    if groups and proc.config:
        proc.set_sensor_groups(groups)
    elif groups:
        print("No config file, can't filter")
    
    proc.process_bag()
    
    if args.convert_ros1:
        if not ROS1_OK:
            print("\nError: ROS1 conversion requested but ROS1 packages not available.")
        else:
            proc.convert_to_ros1(args.ros1_output)

if __name__ == '__main__':
    main()