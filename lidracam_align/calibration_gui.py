#!/usr/bin/env python3
"""
Manual LiDAR-Camera Calibration Tool
Supports ROS1, ROS2, and MCAP files
"""

import sys
import json
import yaml
import numpy as np
import cv2
import os
from pathlib import Path
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional, Tuple, List
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image as PILImage, ImageTk
# Handle different bag formats
HAS_BAGPY = False
try:
    # Try importing rosbag with better error handling
    import sys
    # Temporarily remove ROS2 paths to avoid conflicts
    ros2_paths = [p for p in sys.path if 'ros/foxy' in p]
    for path in ros2_paths:
        sys.path.remove(path)
    
    import rosbag
    HAS_ROS1 = True
    print("ROS1 rosbag support loaded successfully")
    
    # Restore ROS2 paths
    for path in ros2_paths:
        sys.path.append(path)
        
except ImportError as e:
    print(f"ROS1 rosbag import failed: {e}")
    try:
        import bagpy
        HAS_ROS1 = True
        HAS_BAGPY = True
        print("Using bagpy as ROS1 fallback")
    except ImportError:
        HAS_ROS1 = False
        HAS_BAGPY = False
        print("No ROS1 bag support available")

# Replace this section in your script (around line 30-38)

try:
    # Try importing with proper error handling
    import sys
    ros2_path = '/opt/ros/foxy/lib/python3.8/site-packages'
    if ros2_path not in sys.path:
        sys.path.append(ros2_path)
    
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    HAS_ROS2 = True
    print("ROS2 support loaded successfully")
except ImportError as e:
    print(f"ROS2 import failed: {e}")
    HAS_ROS2 = False

try:
    from mcap.reader import make_reader
    from mcap_ros1.reader import DecoderFactory as Ros1DecoderFactory
    from mcap_ros2.reader import DecoderFactory as Ros2DecoderFactory
    from mcap.exceptions import EndOfFile  # Add this import
    HAS_MCAP = True
except ImportError:
    HAS_MCAP = False

# ROS message imports with fallbacks
try:
    from sensor_msgs.msg import Image, PointCloud2, CameraInfo
    import sensor_msgs.point_cloud2 as pc2
    from cv_bridge import CvBridge
    HAS_ROS_MSGS = True
except ImportError:
    print("Warning: ROS messages not available. Using fallback implementations.")
    HAS_ROS_MSGS = False
    
    # Fallback CvBridge implementation
    class CvBridge:
        def imgmsg_to_cv2(self, img_msg, desired_encoding="bgr8"):
            """Convert ROS Image message to OpenCV image"""
            import numpy as np
            import cv2
            
            # Handle CompressedImage messages
            if hasattr(img_msg, 'format') and hasattr(img_msg, 'data'):
                # This is a CompressedImage message
                np_arr = np.frombuffer(img_msg.data, np.uint8)
                img_array = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if img_array is not None and desired_encoding == "bgr8":
                    return img_array
                elif img_array is not None and desired_encoding == "rgb8":
                    return cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
                return img_array
            
            # Handle regular Image messages
            elif hasattr(img_msg, 'data') and hasattr(img_msg, 'encoding'):
                # Handle raw image data
                if img_msg.encoding == 'rgb8':
                    img_array = np.frombuffer(img_msg.data, dtype=np.uint8)
                    img_array = img_array.reshape((img_msg.height, img_msg.width, 3))
                    if desired_encoding == "bgr8":
                        img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
                elif img_msg.encoding == 'bgr8':
                    img_array = np.frombuffer(img_msg.data, dtype=np.uint8)
                    img_array = img_array.reshape((img_msg.height, img_msg.width, 3))
                elif img_msg.encoding == 'mono8':
                    img_array = np.frombuffer(img_msg.data, dtype=np.uint8)
                    img_array = img_array.reshape((img_msg.height, img_msg.width))
                    if desired_encoding == "bgr8":
                        img_array = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)
                else:
                    raise ValueError(f"Unsupported encoding: {img_msg.encoding}")
                return img_array
            return None
    
    # Fallback point cloud processing
    class pc2:
        @staticmethod
        def read_points(cloud, field_names=None, skip_nans=True, uvs=[]):
            """Extract points from PointCloud2 message"""
            import struct
            import numpy as np
            
            if not hasattr(cloud, 'data'):
                return []
                
            # Parse point cloud data
            points = []
            point_step = cloud.point_step
            
            for i in range(0, len(cloud.data), point_step):
                if i + 12 <= len(cloud.data):  # Need at least x,y,z (3 * 4 bytes)
                    x = struct.unpack('f', cloud.data[i:i+4])[0]
                    y = struct.unpack('f', cloud.data[i+4:i+8])[0]
                    z = struct.unpack('f', cloud.data[i+8:i+12])[0]
                    
                    if skip_nans and (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        continue
                        
                    points.append([x, y, z])
            
            return points


@dataclass
class CalibrationParams:
    """Store calibration parameters"""
    rotation: List[List[float]]  # 3x3 rotation matrix
    translation: List[float]     # 3x1 translation vector
    camera_matrix: List[List[float]]  # 3x3 camera intrinsic matrix
    dist_coeffs: List[float]     # distortion coefficients
    image_width: int
    image_height: int


class BagReader:
    """Unified interface for reading different bag formats"""
    
    @staticmethod
    def open_bag(filepath):
        """Detect and open bag file"""
        filepath = Path(filepath)
        ext = filepath.suffix.lower()
        
        if ext == '.bag' and HAS_ROS1:
            return Ros1BagReader(filepath)
        elif ext == '.db3' and HAS_ROS2:
            return Ros2BagReader(filepath)
        elif ext == '.mcap' and HAS_MCAP:
            return McapBagReader(filepath)
        else:
            raise ValueError(f"Unsupported file format or missing dependencies: {ext}")
    
    def read_messages(self, topics):
        """Read messages from specified topics"""
        raise NotImplementedError


class Ros1BagReader(BagReader):
    def __init__(self, filepath):
        self.filepath = str(filepath)
        
        # Try standard rosbag first, then bagpy
        try:
            import rosbag
            self.bag = rosbag.Bag(self.filepath, 'r')
            self.use_bagpy = False
        except ImportError:
            import bagpy
            self.bag = bagpy.bagreader(self.filepath)
            self.use_bagpy = True
            
        # Initialize CvBridge (now always available with fallback)
        self.bridge = CvBridge()
    
    def read_messages(self, topics):
        if self.use_bagpy:
            # bagpy has a different API - it extracts data to CSV files
            # For now, we'll need to handle this differently
            # This is a simplified implementation - bagpy is more complex
            print("Warning: bagpy support is limited. Consider using MCAP format instead.")
            return []
        else:
            # Standard rosbag API
            for topic, msg, t in self.bag.read_messages(topics=topics):
                yield topic, msg, t.to_sec()
    
    def close(self):
        if not self.use_bagpy:
            self.bag.close()


class McapBagReader(BagReader):
    def __init__(self, filepath):
        self.filepath = str(filepath)
        self.bridge = CvBridge()
        
    def read_messages(self, topics):
        """Read messages from MCAP files using multiple approaches"""
        message_count = 0
        topics_filter = set(topics) if topics else None
        
        # Try rosbags with MCAP reader first
        try:
            from rosbags.mcap import Reader as McapReader
            from rosbags.serde import deserialize_cdr
            print("Reading MCAP file using rosbags MCAP reader...")
            
            with McapReader(open(self.filepath, "rb")) as reader:
                # Get available topics
                available_topics = {conn.topic: conn for conn in reader.connections.values()}
                print(f"Available topics in MCAP: {list(available_topics.keys())}")
                
                # Filter topics if specified
                if topics_filter:
                    filtered_connections = [conn for topic, conn in available_topics.items() if topic in topics_filter]
                else:
                    filtered_connections = list(available_topics.values())
                
                print(f"Reading {len(filtered_connections)} connections...")
                
                # Read messages
                for connection, timestamp, data in reader.messages(connections=filtered_connections):
                    topic = connection.topic
                    
                    try:
                        # Deserialize the message
                        decoded_msg = deserialize_cdr(data, connection.msgtype)
                        timestamp_sec = timestamp / 1_000_000_000  # Convert nanoseconds to seconds
                        
                        yield topic, decoded_msg, timestamp_sec
                        message_count += 1
                        
                        if message_count % 10 == 0:
                            print(f"Read {message_count} messages...")
                        
                        if message_count >= 100:  # Limit for performance
                            print(f"Reached message limit ({message_count} messages)")
                            break
                            
                    except Exception as decode_error:
                        print(f"Failed to decode message on topic {topic}: {decode_error}")
                        continue
                        
        except ImportError:
            print("rosbags MCAP reader not available, trying standard rosbags...")
            for topic, decoded_msg, timestamp in self._try_rosbags_standard(topics):
                yield topic, decoded_msg, timestamp
                message_count += 1
            
        except Exception as e:
            print(f"rosbags MCAP reader failed: {e}")
            print("Trying standard rosbags approach...")
            for topic, decoded_msg, timestamp in self._try_rosbags_standard(topics):
                yield topic, decoded_msg, timestamp
                message_count += 1
                
        print(f"Total messages read: {message_count}")
        
    def _try_rosbags_standard(self, topics):
        """Try standard rosbags approach as fallback"""
        message_count = 0
        
        try:
            from rosbags.rosbag2 import Reader
            from rosbags.serde import deserialize_cdr
            print("Trying standard rosbags reader...")
            
            # For single MCAP files, rosbags expects the parent directory
            import os
            bag_dir = os.path.dirname(self.filepath)
            
            with Reader(bag_dir) as reader:
                available_topics = reader.topics
                print(f"Available topics: {list(available_topics.keys())}")
                
                topics_filter = set(topics) if topics else None
                filtered_topics = [t for t in available_topics.keys() if topics_filter is None or t in topics_filter]
                
                for connection, timestamp, rawdata in reader.messages(connections=filtered_topics):
                    topic = connection.topic
                    
                    try:
                        decoded_msg = deserialize_cdr(rawdata, connection.msgtype)
                        timestamp_sec = timestamp * 1e-9
                        
                        yield topic, decoded_msg, timestamp_sec
                        message_count += 1
                        
                        if message_count % 10 == 0:
                            print(f"Read {message_count} messages...")
                        
                        if message_count >= 100:
                            break
                            
                    except Exception as decode_error:
                        print(f"Failed to decode message on topic {topic}: {decode_error}")
                        continue
                        
        except Exception as e:
            print(f"Standard rosbags also failed: {e}")
            print("Trying MCAP fallback...")
            # Final fallback to MCAP approach
            for topic, decoded_msg, timestamp in self._read_with_mcap_fallback(topics):
                yield topic, decoded_msg, timestamp
                message_count += 1
        
    def _read_with_mcap_fallback(self, topics):
        """Fallback MCAP reader approach"""
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
        
        print("Using MCAP fallback approach...")
        topics_filter = set(topics) if topics else None
        
        try:
            with open(self.filepath, "rb") as f:
                reader = make_reader(f, decoder_factories=[DecoderFactory()])
                
                for schema, channel, message, decoded_msg in reader.iter_decoded_messages():
                    topic = channel.topic
                    
                    if topics_filter is None or topic in topics_filter:
                        timestamp = message.publish_time * 1e-9
                        
                        if decoded_msg:
                            yield topic, decoded_msg, timestamp
                            
        except Exception as e:
            print(f"MCAP fallback also failed: {e}")
    
    def close(self):
        pass

class Ros2BagReader(BagReader):
    def __init__(self, filepath):
        self.filepath = str(filepath)
        storage_options = rosbag2_py.StorageOptions(
            uri=self.filepath,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)
        
        # Get topic metadata
        self.topic_types = {}
        topics_and_types = self.reader.get_all_topics_and_types()
        for topic_meta in topics_and_types:
            self.topic_types[topic_meta.name] = topic_meta.type
        
        self.bridge = CvBridge()
    
    def read_messages(self, topics):
        # Filter topics
        topic_filter = rosbag2_py.StorageFilter(topics=topics)
        self.reader.set_filter(topic_filter)
        
        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            if topic in topics:
                # Get message type
                msg_type = self.topic_types.get(topic)
                if msg_type:
                    # Deserialize message
                    msg_class = get_message(msg_type)
                    msg = deserialize_message(data, msg_class)
                    # Convert timestamp
                    timestamp = t * 1e-9  # nanoseconds to seconds
                    yield topic, msg, timestamp
    
    def close(self):
        pass  # Reader closes automatically


class CalibrationGUI:
    def __init__(self):
        print("Initializing CalibrationGUI...")
        self.root = tk.Tk()
        self.root.title("LiDAR-Camera Calibration Tool")
        self.root.geometry("1400x900")
        
        # Data storage
        self.frames = []
        self.current_frame = 0
        
        # Initialize CvBridge (now always available with fallback)
        self.bridge = CvBridge()
        print("CvBridge initialized successfully")
        
        # Calibration parameters
        self.rotation = np.eye(3)
        self.translation = np.zeros(3)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None
        
        # Mouse state
        self.mouse_pressed = False
        self.last_mouse_pos = None
        
        # Build GUI
        self.setup_gui()
        
    def setup_gui(self):
        """Create GUI elements"""
        # Menu bar
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Open Bag/MCAP", command=self.load_bag)
        file_menu.add_separator()
        file_menu.add_command(label="Save Calibration", command=self.save_calibration)
        file_menu.add_command(label="Load Calibration", command=self.load_calibration)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)
        
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Controls
        control_frame = ttk.LabelFrame(main_frame, text="Controls", width=300)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        control_frame.pack_propagate(False)
        
        # Translation controls
        ttk.Label(control_frame, text="Translation (m):", font=('Arial', 10, 'bold')).pack(pady=5)
        
        self.trans_vars = {}
        self.trans_entries = {}
        for i, axis in enumerate(['X', 'Y', 'Z']):
            frame = ttk.Frame(control_frame)
            frame.pack(fill=tk.X, padx=10, pady=2)
            ttk.Label(frame, text=f"{axis}:", width=3).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.trans_vars[axis] = var
            
            # Text entry field
            entry = ttk.Entry(frame, textvariable=var, width=8)
            entry.pack(side=tk.LEFT, padx=(0, 5))
            entry.bind('<Return>', lambda e, a=i: self.update_translation_from_entry(a))
            entry.bind('<FocusOut>', lambda e, a=i: self.update_translation_from_entry(a))
            self.trans_entries[axis] = entry
            
            # Slider
            scale = ttk.Scale(frame, from_=-5.0, to=5.0, variable=var, 
                            command=lambda v, a=i: self.update_translation(a, float(v)))
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # Rotation controls
        ttk.Label(control_frame, text="Rotation (degrees):", font=('Arial', 10, 'bold')).pack(pady=(20, 5))
        
        self.rot_vars = {}
        self.rot_entries = {}
        for i, axis in enumerate(['Roll', 'Pitch', 'Yaw']):
            frame = ttk.Frame(control_frame)
            frame.pack(fill=tk.X, padx=10, pady=2)
            ttk.Label(frame, text=f"{axis}:", width=5).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.rot_vars[axis] = var
            
            # Text entry field
            entry = ttk.Entry(frame, textvariable=var, width=8)
            entry.pack(side=tk.LEFT, padx=(0, 5))
            entry.bind('<Return>', lambda e, a=i: self.update_rotation_from_entry(a))
            entry.bind('<FocusOut>', lambda e, a=i: self.update_rotation_from_entry(a))
            self.rot_entries[axis] = entry
            
            # Slider
            scale = ttk.Scale(frame, from_=-180, to=180, variable=var,
                            command=lambda v, a=i: self.update_rotation(a, float(v)))
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # Frame navigation
        ttk.Label(control_frame, text="Frame Navigation:", font=('Arial', 10, 'bold')).pack(pady=(20, 5))
        nav_frame = ttk.Frame(control_frame)
        nav_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(nav_frame, text="<<", command=self.prev_frame).pack(side=tk.LEFT, padx=2)
        self.frame_label = ttk.Label(nav_frame, text="0/0")
        self.frame_label.pack(side=tk.LEFT, padx=10)
        ttk.Button(nav_frame, text=">>", command=self.next_frame).pack(side=tk.LEFT, padx=2)
        
        # Rolling ball interface
        ttk.Label(control_frame, text="Rolling Ball Control:", font=('Arial', 10, 'bold')).pack(pady=(20, 5))
        
        # Create rolling ball canvas
        self.rolling_ball_frame = ttk.Frame(control_frame)
        self.rolling_ball_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.rolling_ball_canvas = tk.Canvas(self.rolling_ball_frame, width=120, height=120, bg='gray90')
        self.rolling_ball_canvas.pack()
        
        # Initialize rolling ball state
        self.ball_center = (60, 60)
        self.ball_radius = 50
        self.ball_pressed = False
        self.ball_last_pos = None
        
        # Draw the rolling ball
        self.draw_rolling_ball()
        
        # Bind rolling ball events
        self.rolling_ball_canvas.bind("<Button-1>", self.on_ball_press)
        self.rolling_ball_canvas.bind("<B1-Motion>", self.on_ball_drag)
        self.rolling_ball_canvas.bind("<ButtonRelease-1>", self.on_ball_release)
        
        # Add coordinate system info
        coord_info_frame = ttk.Frame(control_frame)
        coord_info_frame.pack(fill=tk.X, padx=10, pady=5)
        
        coord_info = ttk.Label(coord_info_frame, text="Coordinate Systems:\nLiDAR: X=forward, Y=left, Z=up\nCamera: X=right, Y=down, Z=forward", 
                              font=('Courier', 8), justify=tk.LEFT)
        coord_info.pack()
        
        # Heuristic center matching
        ttk.Label(control_frame, text="Auto Alignment:", font=('Arial', 10, 'bold')).pack(pady=(20, 5))
        auto_frame = ttk.Frame(control_frame)
        auto_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(auto_frame, text="Match Centers", command=self.match_centers_heuristic).pack(side=tk.LEFT, padx=2)
        ttk.Button(auto_frame, text="Auto Z-Align", command=self.auto_z_align).pack(side=tk.LEFT, padx=2)
        
        # Point size control
        ttk.Label(control_frame, text="Point Size:", font=('Arial', 10, 'bold')).pack(pady=(20, 5))
        self.point_size_var = tk.IntVar(value=2)
        point_size_frame = ttk.Frame(control_frame)
        point_size_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Scale(point_size_frame, from_=1, to=10, variable=self.point_size_var,
                 command=lambda v: self.update_display()).pack(fill=tk.X)
        
        # Instructions
        instructions = """
Mouse Controls:
- Left Click + Drag: Translate
- Right Click + Drag: Rotate
- Mouse Wheel: Zoom

Keyboard:
- Arrow Keys: Fine translate
- Q/E: Roll
- W/S: Pitch  
- A/D: Yaw
- R: Reset calibration
- Space: Next frame
"""
        ttk.Label(control_frame, text=instructions, justify=tk.LEFT, 
                 font=('Courier', 9)).pack(pady=20, padx=10)
        
        # Right panel - Display (split into camera view and top-down view)
        self.display_frame = ttk.Frame(main_frame)
        self.display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # Create notebook for tabbed views
        self.notebook = ttk.Notebook(self.display_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Camera view tab
        self.camera_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.camera_frame, text="Camera View")
        
        # Create canvas for camera image display
        self.canvas = tk.Canvas(self.camera_frame, bg='black')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Top-down view tab
        self.topdown_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.topdown_frame, text="Top-Down View")
        
        # Create canvas for top-down point cloud display
        self.topdown_canvas = tk.Canvas(self.topdown_frame, bg='gray20')
        self.topdown_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Bind mouse events
        self.canvas.bind("<Button-1>", self.on_mouse_press)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_release)
        self.canvas.bind("<Button-3>", self.on_mouse_press)
        self.canvas.bind("<B3-Motion>", self.on_mouse_rotate)
        self.canvas.bind("<ButtonRelease-3>", self.on_mouse_release)
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)
        self.canvas.bind("<Button-4>", self.on_mouse_wheel)  # Linux
        self.canvas.bind("<Button-5>", self.on_mouse_wheel)  # Linux
        
        # Bind keyboard events
        self.root.bind("<Left>", lambda e: self.translate_by(-0.05, 0, 0))
        self.root.bind("<Right>", lambda e: self.translate_by(0.05, 0, 0))
        self.root.bind("<Up>", lambda e: self.translate_by(0, -0.05, 0))
        self.root.bind("<Down>", lambda e: self.translate_by(0, 0.05, 0))
        self.root.bind("<q>", lambda e: self.rotate_by(-1, 0, 0))
        self.root.bind("<e>", lambda e: self.rotate_by(1, 0, 0))
        self.root.bind("<w>", lambda e: self.rotate_by(0, -1, 0))
        self.root.bind("<s>", lambda e: self.rotate_by(0, 1, 0))
        self.root.bind("<a>", lambda e: self.rotate_by(0, 0, -1))
        self.root.bind("<d>", lambda e: self.rotate_by(0, 0, 1))
        self.root.bind("<r>", lambda e: self.reset_calibration())
        self.root.bind("<space>", lambda e: self.next_frame())
        
    def parse_metadata_yaml(self, mcap_filepath):
        """Parse metadata.yaml file to get topic information"""
        metadata_path = Path(mcap_filepath).parent / "metadata.yaml"
        
        if not metadata_path.exists():
            return set()
        
        try:
            with open(metadata_path, 'r') as f:
                metadata = yaml.safe_load(f)
            
            available_topics = set()
            
            # Extract topics from rosbag2_bagfile_information
            if 'rosbag2_bagfile_information' in metadata:
                bagfile_info = metadata['rosbag2_bagfile_information']
                if 'topics_with_message_count' in bagfile_info:
                    for topic_info in bagfile_info['topics_with_message_count']:
                        if 'topic_metadata' in topic_info:
                            topic_meta = topic_info['topic_metadata']
                            topic_name = topic_meta.get('name', '')
                            topic_type = topic_meta.get('type', '')
                            if topic_name and topic_type:
                                available_topics.add((topic_name, topic_type))
            
            print(f"Found {len(available_topics)} topics from metadata.yaml:")
            for topic, msg_type in sorted(available_topics):
                print(f"  {topic} -> {msg_type}")
            
            return available_topics
            
        except Exception as e:
            print(f"Warning: Could not parse metadata.yaml: {e}")
            return set()
    
    def detect_topics(self, reader):
        """Automatically detect camera, lidar, and camera_info topics from bag file"""
        available_topics = set()
        
        # Try to get topics from different reader types
        try:
            if hasattr(reader, 'reader') and hasattr(reader.reader, 'get_all_topics_and_types'):
                # ROS2 reader
                topics_and_types = reader.reader.get_all_topics_and_types()
                for topic_meta in topics_and_types:
                    available_topics.add((topic_meta.name, topic_meta.type))
            elif hasattr(reader, 'bag') and hasattr(reader.bag, 'get_type_and_topic_info'):
                # ROS1 reader
                info = reader.bag.get_type_and_topic_info()
                for topic, topic_info in info.topics.items():
                    available_topics.add((topic, topic_info.msg_type))
            else:
                # MCAP reader - try metadata.yaml first, then scan messages
                print("Trying to get topics from metadata.yaml...")
                available_topics = self.parse_metadata_yaml(reader.filepath)
                
                if not available_topics:
                    print("Scanning MCAP file for topics...")
                    message_count = 0
                    for topic, msg, t in reader.read_messages([]):
                        available_topics.add((topic, type(msg).__name__))
                        message_count += 1
                        if message_count > 50:  # Limit scanning
                            break
        except Exception as e:
            print(f"Warning: Could not detect topics automatically: {e}")
            # Fallback: try to read some messages
            try:
                message_count = 0
                for topic, msg, t in reader.read_messages([]):
                    available_topics.add((topic, type(msg).__name__))
                    message_count += 1
                    if message_count > 50:
                        break
            except:
                pass
        
        if not available_topics:
            print("No topics found!")
        else:
            print(f"Found {len(available_topics)} topics:")
            for topic, msg_type in sorted(available_topics):
                print(f"  {topic} -> {msg_type}")
        
        # Automatically detect relevant topics
        camera_topics = []
        lidar_topics = []
        camera_info_topics = []
        
        for topic, msg_type in available_topics:
            # Check for camera image topics - prefer debayered over compressed
            if ('Image' in msg_type or 'image' in topic.lower()) and 'camera_info' not in topic.lower():
                # Prioritize debayered images over compressed images
                if 'debayered' in topic.lower():
                    camera_topics.insert(0, topic)  # Insert at beginning for priority
                elif 'compressed' not in topic.lower():
                    camera_topics.append(topic)
            # Check for lidar topics
            elif 'PointCloud2' in msg_type or 'lidar' in topic.lower() or 'points' in topic.lower():
                lidar_topics.append(topic)
            # Check for camera info topics
            elif 'CameraInfo' in msg_type or 'camera_info' in topic.lower():
                camera_info_topics.append(topic)
        
        print(f"\nDetected topics:")
        print(f"  Camera topics: {camera_topics}")
        print(f"  LiDAR topics: {lidar_topics}")
        print(f"  Camera info topics: {camera_info_topics}")
        
        return camera_topics, lidar_topics, camera_info_topics
    
    def select_topics(self, camera_topics, lidar_topics, camera_info_topics):
        """Allow user to select topics if multiple options are available"""
        selected_camera = None
        selected_lidar = None
        selected_info = None
        
        # Select camera topic
        if len(camera_topics) == 1:
            selected_camera = camera_topics[0]
        elif len(camera_topics) > 1:
            # Create a simple selection dialog
            selection_window = tk.Toplevel(self.root)
            selection_window.title("Select Camera Topic")
            selection_window.geometry("400x300")
            
            tk.Label(selection_window, text="Select camera topic:").pack(pady=10)
            
            selected_var = tk.StringVar(value=camera_topics[0])
            for topic in camera_topics:
                tk.Radiobutton(selection_window, text=topic, variable=selected_var, value=topic).pack(anchor='w', padx=20)
            
            def confirm_selection():
                nonlocal selected_camera
                selected_camera = selected_var.get()
                selection_window.destroy()
            
            tk.Button(selection_window, text="OK", command=confirm_selection).pack(pady=10)
            selection_window.wait_window()
        
        # Select lidar topic
        if len(lidar_topics) == 1:
            selected_lidar = lidar_topics[0]
        elif len(lidar_topics) > 1:
            selected_lidar = lidar_topics[0]  # Default to first one
        
        # Select camera info topic
        if len(camera_info_topics) == 1:
            selected_info = camera_info_topics[0]
        elif len(camera_info_topics) > 1:
            # Try to match camera info with selected camera
            if selected_camera:
                # Look for camera info topic that matches the camera topic pattern
                camera_base = selected_camera.replace('/image_raw/debayered', '').replace('/image_raw/compressed', '').replace('/image_raw', '').replace('/image', '')
                print(f"Looking for camera info matching camera base: {camera_base}")
                for info_topic in camera_info_topics:
                    print(f"Checking info topic: {info_topic}")
                    if camera_base in info_topic:
                        selected_info = info_topic
                        print(f"Matched camera info: {selected_info}")
                        break
            if not selected_info:
                selected_info = camera_info_topics[0]  # Default to first one
                print(f"No match found, using default: {selected_info}")
        
        return selected_camera, selected_lidar, selected_info
    
    def find_or_select_calibration_file(self):
        """Find calibration file automatically or let user select it"""
        # Common calibration file locations to search
        search_paths = [
            "/media/nail/payload3/20250826/calibResults/",
            "./calibResults/",
            "../calibResults/",
            "../../calibResults/",
            "./",
        ]
        
        # Look for camchain.yaml files
        for search_path in search_paths:
            if os.path.exists(search_path):
                for file in os.listdir(search_path):
                    if "camchain.yaml" in file and not file.startswith('.'):
                        full_path = os.path.join(search_path, file)
                        print(f"Found calibration file: {full_path}")
                        return full_path
        
        # If not found, ask user to select
        print("Calibration file not found automatically, asking user to select...")
        calib_file = filedialog.askopenfilename(
            title="Select Camera Calibration File",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
            initialdir="/media/nail/payload3/20250826/calibResults/"
        )
        
        if not calib_file:
            raise ValueError("No calibration file selected")
            
        return calib_file

    def load_bag(self):
        """Load ROS bag or MCAP file"""
        filepath = filedialog.askopenfilename(
            title="Select bag file",
            filetypes=[("ROS1 bags", "*.bag"),
                       ("ROS2 bags", "*.db3"),
                       ("MCAP", "*.mcap"),
                       ("All files", "*.*")]
        )
        
        if not filepath:
            return
            
        try:
            # Clear existing data
            self.frames = []
            self.current_frame = 0
            
            print(f"Loading bag file: {filepath}")
            
            # Read bag file
            reader = BagReader.open_bag(filepath)
            
            # Automatically detect topics
            camera_topics, lidar_topics, camera_info_topics = self.detect_topics(reader)
            
            # Check if we found the required topics
            if not camera_topics:
                messagebox.showerror("Error", "No camera image topics found in bag file")
                reader.close()
                return
            
            if not lidar_topics:
                messagebox.showerror("Error", "No LiDAR point cloud topics found in bag file")
                reader.close()
                return
            
            # Select topics (automatically or via user selection)
            camera_topic, lidar_topic, info_topic = self.select_topics(
                camera_topics, lidar_topics, camera_info_topics
            )
            
            if not camera_topic or not lidar_topic:
                messagebox.showerror("Error", "Could not select required topics")
                reader.close()
                return
            
            print(f"Using topics:")
            print(f"  Camera: {camera_topic}")
            print(f"  LiDAR: {lidar_topic}")
            print(f"  Camera Info: {info_topic}")
            
            # Topics to read
            topics_to_read = [camera_topic, lidar_topic]
            if info_topic:
                topics_to_read.append(info_topic)
            
            # Read messages with limits for performance
            messages = {}
            message_count = 0
            max_messages_per_topic = 50  # Limit messages per topic
            
            print(f"Reading messages from topics: {topics_to_read}")
            for topic, msg, t in reader.read_messages(topics_to_read):
                if topic not in messages:
                    messages[topic] = []
                
                # Limit messages per topic for performance
                if len(messages[topic]) < max_messages_per_topic:
                    messages[topic].append((msg, t))
                    message_count += 1
                    
                    if message_count % 10 == 0:
                        print(f"Read {message_count} messages...")
                
                # Stop if we have enough messages for all topics
                all_topics_have_enough = all(
                    topic in messages and len(messages[topic]) >= min(10, max_messages_per_topic) 
                    for topic in topics_to_read
                )
                if all_topics_have_enough and message_count > 100:
                    print(f"Stopping early - collected enough messages ({message_count} total)")
                    break
            
            print(f"Finished reading {message_count} messages")
            
            # Load actual camera calibration parameters from calibration file
            print(f"Loading camera calibration from calibResults...")
            
            # Determine which camera we're using (cam0 or cam1)
            cam_num = "cam0" if "cam0" in camera_topic else "cam1"
            print(f"Using camera: {cam_num}")
            
            # Load calibration file - try to find it or ask user
            calib_file = self.find_or_select_calibration_file()
            
            try:
                import yaml
                with open(calib_file, 'r') as f:
                    calib_data = yaml.safe_load(f)
                
                if cam_num in calib_data:
                    cam_calib = calib_data[cam_num]
                    
                    # Extract intrinsics [fx, fy, cx, cy]
                    intrinsics = cam_calib['intrinsics']
                    fx, fy, cx, cy = intrinsics
                    
                    # Build camera matrix
                    self.camera_matrix = np.array([
                        [fx, 0.0, cx],
                        [0.0, fy, cy],
                        [0.0, 0.0, 1.0]
                    ])
                    
                    # Extract distortion coefficients
                    self.dist_coeffs = np.array(cam_calib['distortion_coeffs'])
                    
                    # Extract image resolution
                    resolution = cam_calib['resolution']
                    self.image_size = (resolution[0], resolution[1])
                    
                    print(f"Loaded calibration for {cam_num}:")
                    print(f"  Camera matrix:\n{self.camera_matrix}")
                    print(f"  Distortion coeffs: {self.dist_coeffs}")
                    print(f"  Image size: {self.image_size}")
                    
                else:
                    raise ValueError(f"Camera {cam_num} not found in calibration file")
                    
            except Exception as e:
                print(f"Error loading calibration file: {e}")
                print("Falling back to bag camera info...")
                
                # Fallback to bag camera info
                if info_topic in messages and messages[info_topic]:
                    info_msg = messages[info_topic][0][0]
                    if hasattr(info_msg, 'width') and hasattr(info_msg, 'height'):
                        if info_msg.width > 0 and info_msg.height > 0:
                            self.image_size = (info_msg.width, info_msg.height)
                        else:
                            self.image_size = (1224, 1024)  # Default from calib file
                    else:
                        self.image_size = (1224, 1024)
                    
                    # Use default reasonable camera matrix if bag info is empty
                    self.camera_matrix = np.array([
                        [2178.5, 0.0, 673.2],
                        [0.0, 2036.2, 519.6],
                        [0.0, 0.0, 1.0]
                    ])
                    self.dist_coeffs = np.array([-0.406, 1.521, 0.012, -0.014])
                    
                    print(f"Using fallback camera parameters:")
                    print(f"  Camera matrix:\n{self.camera_matrix}")
                    print(f"  Image size: {self.image_size}")
            
            # Synchronize frames
            if camera_topic in messages and lidar_topic in messages:
                camera_msgs = messages[camera_topic]
                lidar_msgs = messages[lidar_topic]
                
                # Simple synchronization - pair closest timestamps
                for cam_msg, cam_t in camera_msgs[:10]:  # Limit frames for performance
                    # Find closest lidar message
                    best_lidar = min(lidar_msgs, key=lambda x: abs(x[1] - cam_t))
                    if abs(best_lidar[1] - cam_t) < 0.1:  # 100ms tolerance
                        self.frames.append({
                            'image': self.bridge.imgmsg_to_cv2(cam_msg, 'bgr8'),
                            'points': self.extract_points(best_lidar[0])
                        })
                
                if self.frames:
                    self.frame_label.config(text=f"1/{len(self.frames)}")
                    self.update_display()
                    messagebox.showinfo("Success", f"Loaded {len(self.frames)} synchronized frames")
                else:
                    messagebox.showerror("Error", "No synchronized frames found")
                    
            reader.close()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load bag: {str(e)}")
    
    def extract_points(self, pc2_msg):
        """Extract xyz points from PointCloud2 message"""
        points = []
        try:
            for point in pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            points_array = np.array(points)
            print(f"Extracted {len(points_array)} points from point cloud")
            if len(points_array) > 0:
                print(f"Point cloud bounds: X[{points_array[:, 0].min():.2f}, {points_array[:, 0].max():.2f}], "
                      f"Y[{points_array[:, 1].min():.2f}, {points_array[:, 1].max():.2f}], "
                      f"Z[{points_array[:, 2].min():.2f}, {points_array[:, 2].max():.2f}]")
            return points_array
        except Exception as e:
            print(f"Error extracting points: {e}")
            return np.array([])
    
    def project_points(self, points_3d):
        """Project 3D points to 2D image plane using proper coordinate transformations"""
        if self.camera_matrix is None or len(points_3d) == 0:
            print("No camera matrix or no points to project")
            return np.array([])
        
        print(f"Projecting {len(points_3d)} points")
        
        # Create 4x4 transformation matrix (LiDAR to Camera)
        # Standard convention: T = [R t; 0 1] where R is 3x3 rotation, t is 3x1 translation
        T_lidar_to_camera = np.eye(4)
        T_lidar_to_camera[:3, :3] = self.rotation
        T_lidar_to_camera[:3, 3] = self.translation
        
        print(f"Transformation matrix (LiDAR->Camera):\n{T_lidar_to_camera}")
        
        # Convert points to homogeneous coordinates
        points_homogeneous = np.hstack([points_3d, np.ones((len(points_3d), 1))])
        
        # Apply transformation: P_camera = T * P_lidar
        points_camera = (T_lidar_to_camera @ points_homogeneous.T).T
        points_camera = points_camera[:, :3]  # Convert back to 3D
        
        print(f"After transformation: {len(points_camera)} points")
        if len(points_camera) > 0:
            print(f"Camera frame bounds: X[{points_camera[:, 0].min():.2f}, {points_camera[:, 0].max():.2f}], "
                  f"Y[{points_camera[:, 1].min():.2f}, {points_camera[:, 1].max():.2f}], "
                  f"Z[{points_camera[:, 2].min():.2f}, {points_camera[:, 2].max():.2f}]")
        
        # Remove points behind camera (Z <= 0 in camera coordinates)
        # Camera coordinate system: X=right, Y=down, Z=forward
        valid_mask = points_camera[:, 2] > 0.1  # Points must be in front of camera
        points_camera = points_camera[valid_mask]
        print(f"After removing points behind camera: {len(points_camera)} points")
        
        if len(points_camera) == 0:
            print("No points in front of camera")
            return np.array([])
        
        # Project to image plane using pinhole camera model
        # P_image = K * P_camera where K is camera intrinsic matrix
        points_2d_homogeneous = self.camera_matrix @ points_camera.T
        
        # Avoid division by zero
        z_coords = points_2d_homogeneous[2]
        valid_z = np.abs(z_coords) > 1e-6
        if not np.any(valid_z):
            print("All points have zero Z coordinate after projection")
            return np.array([])
        
        points_2d_homogeneous = points_2d_homogeneous[:, valid_z]
        points_camera = points_camera[valid_z]
        
        # Convert from homogeneous to 2D image coordinates
        points_2d = points_2d_homogeneous[:2] / points_2d_homogeneous[2]
        points_2d = points_2d.T  # Shape: (N, 2)
        print(f"After projection: {len(points_2d)} points")
        
        # Filter points within image bounds
        if self.image_size:
            valid_mask = (points_2d[:, 0] >= 0) & (points_2d[:, 0] < self.image_size[0]) & \
                        (points_2d[:, 1] >= 0) & (points_2d[:, 1] < self.image_size[1])
            
            # Add depth for coloring (Z coordinate in camera frame)
            depths = points_camera[valid_mask, 2]
            points_2d = points_2d[valid_mask]
            print(f"After filtering within image bounds: {len(points_2d)} points")
        else:
            depths = points_camera[:, 2]
            print(f"No image size filtering, keeping all {len(points_2d)} points")
        
        if len(points_2d) > 0:
            result = np.column_stack([points_2d, depths])
            print(f"Final projected points: {len(result)} points")
            return result
        else:
            print("No points within image bounds")
            return np.array([])
    
    def update_display(self):
        """Update the display with current frame and calibration"""
        if not self.frames or self.current_frame >= len(self.frames):
            return
            
        frame = self.frames[self.current_frame]
        image = frame['image'].copy()
        points_3d = frame['points']
        
        # Project points
        projected = self.project_points(points_3d)
        
        if len(projected) > 0:
            # Color points by depth
            depths = projected[:, 2]
            depth_min = depths.min()
            depth_max = depths.max()
            
            # Avoid division by zero
            if depth_max - depth_min > 0:
                depth_norm = (depths - depth_min) / (depth_max - depth_min)
            else:
                depth_norm = np.zeros_like(depths)
            
            # Draw points
            point_size = self.point_size_var.get()
            
            # Create a colormap array
            colormap_array = np.uint8(depth_norm * 255).reshape(-1, 1)
            colors = cv2.applyColorMap(colormap_array, cv2.COLORMAP_JET)
            
            for i, (x, y, d) in enumerate(projected):
                color = tuple(map(int, colors[i, 0]))
                cv2.circle(image, (int(x), int(y)), point_size, color, -1)
        
        # Convert to PhotoImage and display
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_pil = PILImage.fromarray(image_rgb)
        
        # Resize to fit canvas
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        if canvas_width > 1 and canvas_height > 1:
            image_pil.thumbnail((canvas_width, canvas_height), PILImage.Resampling.LANCZOS)
        
        self.photo = ImageTk.PhotoImage(image_pil)
        self.canvas.delete("all")
        self.canvas.create_image(canvas_width//2, canvas_height//2, image=self.photo)
        
        # Update top-down view
        self.update_topdown_view()
    
    def update_translation(self, axis, value):
        """Update translation from GUI"""
        self.translation[axis] = value
        self.update_display()
    
    def update_translation_from_entry(self, axis):
        """Update translation from text entry field"""
        try:
            axis_names = ['X', 'Y', 'Z']
            axis_name = axis_names[axis]
            value = self.trans_vars[axis_name].get()
            self.translation[axis] = value
            self.update_display()
        except (ValueError, tk.TclError):
            # Reset to current value if invalid input
            axis_names = ['X', 'Y', 'Z']
            axis_name = axis_names[axis]
            self.trans_vars[axis_name].set(self.translation[axis])
    
    def update_topdown_view(self):
        """Update the top-down point cloud view"""
        if not self.frames or self.current_frame >= len(self.frames):
            return
            
        # Get current frame points
        points_3d = self.frames[self.current_frame]['points']
        if len(points_3d) == 0:
            return
            
        # Apply current transformation
        points_transformed = (self.rotation @ points_3d.T).T + self.translation
        
        # Get canvas dimensions
        canvas_width = self.topdown_canvas.winfo_width()
        canvas_height = self.topdown_canvas.winfo_height()
        
        if canvas_width <= 1 or canvas_height <= 1:
            return
            
        # Clear canvas
        self.topdown_canvas.delete("all")
        
        # Project to X-Y plane (top-down view)
        x_coords = points_transformed[:, 0]
        y_coords = points_transformed[:, 1]
        z_coords = points_transformed[:, 2]
        
        if len(x_coords) == 0:
            return
            
        # Calculate bounds with padding
        x_min, x_max = x_coords.min(), x_coords.max()
        y_min, y_max = y_coords.min(), y_coords.max()
        
        # Add padding
        x_range = x_max - x_min
        y_range = y_max - y_min
        padding = 0.1
        x_min -= x_range * padding
        x_max += x_range * padding
        y_min -= y_range * padding
        y_max += y_range * padding
        
        # Scale to canvas
        if x_range > 0 and y_range > 0:
            scale_x = (canvas_width - 40) / (x_max - x_min)
            scale_y = (canvas_height - 40) / (y_max - y_min)
            scale = min(scale_x, scale_y)
            
            # Transform coordinates to canvas space
            canvas_x = (x_coords - x_min) * scale + 20
            canvas_y = canvas_height - ((y_coords - y_min) * scale + 20)  # Flip Y axis
            
            # Color by height (Z coordinate)
            if len(z_coords) > 0:
                z_min, z_max = z_coords.min(), z_coords.max()
                if z_max - z_min > 0:
                    z_norm = (z_coords - z_min) / (z_max - z_min)
                else:
                    z_norm = np.zeros_like(z_coords)
                
                # Draw points
                point_size = max(1, self.point_size_var.get() // 2)
                
                for i in range(len(canvas_x)):
                    # Color based on height
                    color_val = int(z_norm[i] * 255)
                    if color_val < 128:
                        # Blue to green
                        color = f"#{0:02x}{color_val*2:02x}{255-color_val*2:02x}"
                    else:
                        # Green to red
                        color_val = (color_val - 128) * 2
                        color = f"#{color_val:02x}{255-color_val:02x}{0:02x}"
                    
                    x, y = int(canvas_x[i]), int(canvas_y[i])
                    self.topdown_canvas.create_oval(
                        x-point_size, y-point_size, 
                        x+point_size, y+point_size,
                        fill=color, outline=""
                    )
        
        # Draw coordinate axes
        center_x, center_y = canvas_width // 2, canvas_height // 2
        axis_length = 30
        
        # X axis (red)
        self.topdown_canvas.create_line(
            center_x, center_y, center_x + axis_length, center_y,
            fill="red", width=2
        )
        self.topdown_canvas.create_text(
            center_x + axis_length + 10, center_y, text="X", fill="red"
        )
        
        # Y axis (green)
        self.topdown_canvas.create_line(
            center_x, center_y, center_x, center_y - axis_length,
            fill="green", width=2
        )
        self.topdown_canvas.create_text(
            center_x, center_y - axis_length - 10, text="Y", fill="green"
        )
        
        # Add camera position indicator
        cam_x = (-self.translation[0] - x_min) * scale + 20 if x_range > 0 else center_x
        cam_y = canvas_height - ((-self.translation[1] - y_min) * scale + 20) if y_range > 0 else center_y
        
        # Draw camera as triangle
        cam_size = 8
        self.topdown_canvas.create_polygon(
            cam_x, cam_y - cam_size,
            cam_x - cam_size, cam_y + cam_size,
            cam_x + cam_size, cam_y + cam_size,
            fill="yellow", outline="black", width=2
        )
        self.topdown_canvas.create_text(
            cam_x, cam_y + cam_size + 15, text="Camera", fill="yellow"
        )
    
    def update_rotation(self, axis, value):
        """Update rotation from GUI"""
        angles = [self.rot_vars['Roll'].get(), 
                 self.rot_vars['Pitch'].get(), 
                 self.rot_vars['Yaw'].get()]
        angles = np.deg2rad(angles)
        
        # Create rotation matrix from Euler angles
        R_x = np.array([[1, 0, 0],
                       [0, np.cos(angles[0]), -np.sin(angles[0])],
                       [0, np.sin(angles[0]), np.cos(angles[0])]])
        
        R_y = np.array([[np.cos(angles[1]), 0, np.sin(angles[1])],
                       [0, 1, 0],
                       [-np.sin(angles[1]), 0, np.cos(angles[1])]])
        
        R_z = np.array([[np.cos(angles[2]), -np.sin(angles[2]), 0],
                       [np.sin(angles[2]), np.cos(angles[2]), 0],
                       [0, 0, 1]])
        
        self.rotation = R_z @ R_y @ R_x
        self.update_display()
    
    def update_rotation_from_entry(self, axis):
        """Update rotation from text entry field"""
        try:
            axis_names = ['Roll', 'Pitch', 'Yaw']
            axis_name = axis_names[axis]
            value = self.rot_vars[axis_name].get()
            # Clamp to valid range
            value = max(-180, min(180, value))
            self.rot_vars[axis_name].set(value)
            self.update_rotation(axis, value)
        except (ValueError, tk.TclError):
            # Reset to current value if invalid input
            axis_names = ['Roll', 'Pitch', 'Yaw']
            axis_name = axis_names[axis]
            # Convert current rotation matrix back to Euler angles
            sy = np.sqrt(self.rotation[0, 0]**2 + self.rotation[1, 0]**2)
            singular = sy < 1e-6
            
            if not singular:
                x = np.arctan2(self.rotation[2, 1], self.rotation[2, 2])
                y = np.arctan2(-self.rotation[2, 0], sy)
                z = np.arctan2(self.rotation[1, 0], self.rotation[0, 0])
            else:
                x = np.arctan2(-self.rotation[1, 2], self.rotation[1, 1])
                y = np.arctan2(-self.rotation[2, 0], sy)
                z = 0
            
            angles_deg = [np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)]
            self.rot_vars[axis_name].set(angles_deg[axis])
    
    def translate_by(self, dx, dy, dz):
        """Translate by relative amount"""
        self.translation += np.array([dx, dy, dz])
        self.trans_vars['X'].set(self.translation[0])
        self.trans_vars['Y'].set(self.translation[1])
        self.trans_vars['Z'].set(self.translation[2])
        self.update_display()
    
    def rotate_by(self, droll, dpitch, dyaw):
        """Rotate by relative amount"""
        self.rot_vars['Roll'].set(self.rot_vars['Roll'].get() + droll)
        self.rot_vars['Pitch'].set(self.rot_vars['Pitch'].get() + dpitch)
        self.rot_vars['Yaw'].set(self.rot_vars['Yaw'].get() + dyaw)
        self.update_rotation(0, 0)  # Trigger update
    
    def reset_calibration(self):
        """Reset calibration to identity"""
        self.rotation = np.eye(3)
        self.translation = np.zeros(3)
        
        for var in self.trans_vars.values():
            var.set(0.0)
        for var in self.rot_vars.values():
            var.set(0.0)
            
        self.update_display()
    
    def on_mouse_press(self, event):
        """Handle mouse press"""
        self.mouse_pressed = True
        self.last_mouse_pos = (event.x, event.y)
    
    def on_mouse_release(self, event):
        """Handle mouse release"""
        self.mouse_pressed = False
        self.last_mouse_pos = None
    
    def on_mouse_drag(self, event):
        """Handle mouse drag for translation"""
        if self.mouse_pressed and self.last_mouse_pos:
            dx = event.x - self.last_mouse_pos[0]
            dy = event.y - self.last_mouse_pos[1]
            
            # Scale based on canvas size
            scale = 0.01
            self.translate_by(dx * scale, dy * scale, 0)
            
            self.last_mouse_pos = (event.x, event.y)
    
    def on_mouse_rotate(self, event):
        """Handle mouse drag for rotation"""
        if self.mouse_pressed and self.last_mouse_pos:
            dx = event.x - self.last_mouse_pos[0]
            dy = event.y - self.last_mouse_pos[1]
            
            # Scale based on canvas size
            scale = 0.5
            self.rotate_by(0, dy * scale, -dx * scale)
            
            self.last_mouse_pos = (event.x, event.y)
    
    def on_mouse_wheel(self, event):
        """Handle mouse wheel for zoom"""
        # Handle different platforms
        if event.num == 4 or event.delta > 0:
            self.translate_by(0, 0, -0.1)
        elif event.num == 5 or event.delta < 0:
            self.translate_by(0, 0, 0.1)
    
    def draw_rolling_ball(self):
        """Draw the rolling ball interface"""
        self.rolling_ball_canvas.delete("all")
        
        # Draw outer circle
        x, y = self.ball_center
        r = self.ball_radius
        self.rolling_ball_canvas.create_oval(
            x - r, y - r, x + r, y + r,
            outline="black", width=2, fill="lightblue"
        )
        
        # Draw grid lines
        for i in range(-2, 3):
            # Vertical lines
            line_x = x + i * r // 4
            if x - r < line_x < x + r:
                y_offset = np.sqrt(r**2 - (line_x - x)**2)
                self.rolling_ball_canvas.create_line(
                    line_x, y - y_offset, line_x, y + y_offset,
                    fill="gray", width=1
                )
            
            # Horizontal lines
            line_y = y + i * r // 4
            if y - r < line_y < y + r:
                x_offset = np.sqrt(r**2 - (line_y - y)**2)
                self.rolling_ball_canvas.create_line(
                    x - x_offset, line_y, x + x_offset, line_y,
                    fill="gray", width=1
                )
        
        # Draw center point
        self.rolling_ball_canvas.create_oval(
            x - 3, y - 3, x + 3, y + 3,
            fill="red", outline="darkred"
        )
        
        # Draw current rotation indicator
        current_roll = np.deg2rad(self.rot_vars['Roll'].get())
        current_pitch = np.deg2rad(self.rot_vars['Pitch'].get())
        
        # Map rotation to ball position
        ball_x = x + current_pitch * r / (np.pi / 2)
        ball_y = y - current_roll * r / (np.pi / 2)
        
        # Clamp to circle
        dx = ball_x - x
        dy = ball_y - y
        dist = np.sqrt(dx**2 + dy**2)
        if dist > r - 5:
            scale = (r - 5) / dist
            ball_x = x + dx * scale
            ball_y = y + dy * scale
        
        # Draw rotation indicator
        self.rolling_ball_canvas.create_oval(
            ball_x - 5, ball_y - 5, ball_x + 5, ball_y + 5,
            fill="orange", outline="darkorange", width=2
        )
    
    def on_ball_press(self, event):
        """Handle rolling ball press"""
        self.ball_pressed = True
        self.ball_last_pos = (event.x, event.y)
    
    def on_ball_release(self, event):
        """Handle rolling ball release"""
        self.ball_pressed = False
        self.ball_last_pos = None
    
    def on_ball_drag(self, event):
        """Handle rolling ball drag"""
        if not self.ball_pressed or not self.ball_last_pos:
            return
        
        # Calculate movement delta from last position
        dx = event.x - self.ball_last_pos[0]
        dy = event.y - self.ball_last_pos[1]
        
        # Convert to rotation angles with sensitivity control
        sensitivity = 0.5  # Adjust sensitivity
        pitch_delta = dx * sensitivity
        roll_delta = -dy * sensitivity  # Negative for intuitive control
        
        # Update rotation variables
        current_pitch = self.rot_vars['Pitch'].get()
        current_roll = self.rot_vars['Roll'].get()
        
        new_pitch = max(-180, min(180, current_pitch + pitch_delta))
        new_roll = max(-180, min(180, current_roll + roll_delta))
        
        self.rot_vars['Pitch'].set(new_pitch)
        self.rot_vars['Roll'].set(new_roll)
        
        # Update rotation matrix
        self.update_rotation(0, 0)
        
        # Update last position
        self.ball_last_pos = (event.x, event.y)
        
        # Redraw ball
        self.draw_rolling_ball()
    
    def match_centers_heuristic(self):
        """Improved heuristic center matching using proper coordinate systems"""
        if not self.frames or self.current_frame >= len(self.frames):
            messagebox.showwarning("Warning", "No frame data available")
            return
        
        points_3d = self.frames[self.current_frame]['points']
        if len(points_3d) == 0:
            messagebox.showwarning("Warning", "No point cloud data available")
            return
        
        try:
            print(f"\n=== CENTER MATCHING HEURISTIC ===")
            print(f"Point cloud stats: {len(points_3d)} points")
            print(f"LiDAR frame bounds: X[{points_3d[:, 0].min():.2f}, {points_3d[:, 0].max():.2f}], "
                  f"Y[{points_3d[:, 1].min():.2f}, {points_3d[:, 1].max():.2f}], "
                  f"Z[{points_3d[:, 2].min():.2f}, {points_3d[:, 2].max():.2f}]")
            
            # Filter points to reasonable range (remove outliers)
            # Typical automotive LiDAR range: 0.5m to 100m
            distances = np.linalg.norm(points_3d, axis=1)
            valid_mask = (distances > 0.5) & (distances < 100) & \
                        (np.abs(points_3d[:, 0]) < 50) & (np.abs(points_3d[:, 1]) < 50)
            filtered_points = points_3d[valid_mask]
            
            if len(filtered_points) < 100:
                messagebox.showwarning("Warning", "Not enough valid points for alignment")
                return
            
            print(f"After filtering: {len(filtered_points)} points")
            
            # Calculate robust centroid using median (less sensitive to outliers)
            centroid = np.median(filtered_points, axis=0)
            print(f"LiDAR centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")
            
            if self.camera_matrix is not None and self.image_size is not None:
                # Estimate initial transformation based on typical sensor setup
                # Common LiDAR-camera setup: LiDAR on top, camera below/forward
                
                # Estimate viewing distance (median distance to points)
                viewing_distance = np.median(np.linalg.norm(filtered_points, axis=1))
                print(f"Estimated viewing distance: {viewing_distance:.2f}m")
                
                # Initial guess for transformation
                # Assume camera is looking forward (positive Z in camera frame)
                # and LiDAR coordinate system needs to be aligned
                
                # Translation: move LiDAR origin to camera position
                # Camera typically positioned to view the scene center
                self.translation = np.array([
                    -centroid[0] * 0.1,  # Small X adjustment
                    -centroid[1] * 0.1,  # Small Y adjustment  
                    viewing_distance * 0.3  # Position camera to view the scene
                ])
                
                # Initial rotation: align coordinate systems
                # Common setup: LiDAR Z-up, Camera Z-forward
                # Apply small initial rotation to align better
                initial_pitch = -5.0  # degrees, camera looking slightly down
                initial_yaw = 0.0
                initial_roll = 0.0
                
                self.rot_vars['Roll'].set(initial_roll)
                self.rot_vars['Pitch'].set(initial_pitch)
                self.rot_vars['Yaw'].set(initial_yaw)
                
                # Update rotation matrix
                self.update_rotation(0, 0)
                
                print(f"Applied initial transformation:")
                print(f"  Translation: [{self.translation[0]:.3f}, {self.translation[1]:.3f}, {self.translation[2]:.3f}]")
                print(f"  Rotation (RPY): [{initial_roll:.1f}, {initial_pitch:.1f}, {initial_yaw:.1f}] degrees")
                
                # Update GUI
                for i, axis in enumerate(['X', 'Y', 'Z']):
                    self.trans_vars[axis].set(self.translation[i])
                
                self.update_display()
                self.draw_rolling_ball()
                
                messagebox.showinfo("Success", 
                    f"Initial alignment applied:\n"
                    f"Translation: [{self.translation[0]:.2f}, {self.translation[1]:.2f}, {self.translation[2]:.2f}]\n"
                    f"Rotation: [{initial_roll:.1f}°, {initial_pitch:.1f}°, {initial_yaw:.1f}°]\n\n"
                    f"Fine-tune using sliders or rolling ball control.")
            else:
                messagebox.showerror("Error", "Camera calibration data not available")
                
        except Exception as e:
            print(f"Error in match_centers_heuristic: {e}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to match centers: {str(e)}")
    
    def auto_z_align(self):
        """Automatically align Z-axis based on ground plane detection"""
        if not self.frames or self.current_frame >= len(self.frames):
            messagebox.showwarning("Warning", "No frame data available")
            return
        
        points_3d = self.frames[self.current_frame]['points']
        if len(points_3d) == 0:
            messagebox.showwarning("Warning", "No point cloud data available")
            return
        
        try:
            print(f"Auto Z-align: Processing {len(points_3d)} points")
            
            # Filter points to reasonable range
            valid_mask = (np.abs(points_3d[:, 0]) < 50) & (np.abs(points_3d[:, 1]) < 50) & (points_3d[:, 2] > -10) & (points_3d[:, 2] < 50)
            filtered_points = points_3d[valid_mask]
            
            if len(filtered_points) < 50:
                messagebox.showwarning("Warning", "Not enough valid points for Z-alignment")
                return
            
            print(f"After filtering: {len(filtered_points)} points")
            
            # Simple approach: align with the dominant horizontal plane
            # Use PCA to find the dominant plane
            centered_points = filtered_points - np.mean(filtered_points, axis=0)
            
            # Compute covariance matrix
            cov_matrix = np.cov(centered_points.T)
            
            # Find eigenvectors (principal components)
            eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
            
            # The eigenvector with smallest eigenvalue is the normal to the dominant plane
            normal_idx = np.argmin(eigenvalues)
            plane_normal = eigenvectors[:, normal_idx]
            
            # Ensure normal points upward (positive Z component)
            if plane_normal[2] < 0:
                plane_normal = -plane_normal
            
            print(f"Detected plane normal: [{plane_normal[0]:.3f}, {plane_normal[1]:.3f}, {plane_normal[2]:.3f}]")
            
            # Calculate rotation to align normal with Z-axis
            target_normal = np.array([0, 0, 1])
            
            # Calculate rotation axis and angle
            rotation_axis = np.cross(plane_normal, target_normal)
            rotation_angle = np.arccos(np.clip(np.dot(plane_normal, target_normal), -1, 1))
            
            print(f"Rotation angle: {np.rad2deg(rotation_angle):.2f} degrees")
            
            if rotation_angle > 0.01:  # Only apply if significant rotation needed
                if np.linalg.norm(rotation_axis) > 1e-6:
                    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                    
                    # Convert to rotation matrix using Rodrigues' formula
                    K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                 [rotation_axis[2], 0, -rotation_axis[0]],
                                 [-rotation_axis[1], rotation_axis[0], 0]])
                    
                    R_align = np.eye(3) + np.sin(rotation_angle) * K + (1 - np.cos(rotation_angle)) * np.dot(K, K)
                    
                    # Apply rotation
                    self.rotation = R_align @ self.rotation
                    
                    # Convert back to Euler angles for GUI
                    sy = np.sqrt(self.rotation[0, 0]**2 + self.rotation[1, 0]**2)
                    singular = sy < 1e-6
                    
                    if not singular:
                        x = np.arctan2(self.rotation[2, 1], self.rotation[2, 2])
                        y = np.arctan2(-self.rotation[2, 0], sy)
                        z = np.arctan2(self.rotation[1, 0], self.rotation[0, 0])
                    else:
                        x = np.arctan2(-self.rotation[1, 2], self.rotation[1, 1])
                        y = np.arctan2(-self.rotation[2, 0], sy)
                        z = 0
                    
                    self.rot_vars['Roll'].set(np.rad2deg(x))
                    self.rot_vars['Pitch'].set(np.rad2deg(y))
                    self.rot_vars['Yaw'].set(np.rad2deg(z))
                    
                    print(f"Applied rotation: Roll={np.rad2deg(x):.2f}, Pitch={np.rad2deg(y):.2f}, Yaw={np.rad2deg(z):.2f}")
                    
                    self.update_display()
                    self.draw_rolling_ball()
                    messagebox.showinfo("Success", f"Z-axis aligned with dominant plane (rotated {np.rad2deg(rotation_angle):.1f}°)")
                else:
                    messagebox.showinfo("Info", "Plane normal is already aligned with Z-axis")
            else:
                messagebox.showinfo("Info", "Plane already well-aligned with Z-axis")
                
        except Exception as e:
            print(f"Error in auto_z_align: {e}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Error", f"Failed to auto-align Z-axis: {str(e)}")
    
    def next_frame(self):
        """Go to next frame"""
        if self.frames and self.current_frame < len(self.frames) - 1:
            self.current_frame += 1
            self.frame_label.config(text=f"{self.current_frame + 1}/{len(self.frames)}")
            self.update_display()
    
    def prev_frame(self):
        """Go to previous frame"""
        if self.frames and self.current_frame > 0:
            self.current_frame -= 1
            self.frame_label.config(text=f"{self.current_frame + 1}/{len(self.frames)}")
            self.update_display()
    
    def save_calibration(self):
        """Save calibration parameters"""
        if self.camera_matrix is None:
            messagebox.showerror("Error", "No calibration data to save")
            return
            
        filepath = filedialog.asksaveasfilename(
            title="Save Calibration",
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml"), ("JSON", "*.json")]
        )
        
        if not filepath:
            return
            
        try:
            params = CalibrationParams(
                rotation=self.rotation.tolist(),
                translation=self.translation.tolist(),
                camera_matrix=self.camera_matrix.tolist(),
                dist_coeffs=self.dist_coeffs.tolist() if self.dist_coeffs is not None else [],
                image_width=self.image_size[0],
                image_height=self.image_size[1]
            )
            
            if filepath.endswith('.json'):
                with open(filepath, 'w') as f:
                    json.dump(asdict(params), f, indent=2)
            else:
                # For YAML, add formatted rotation matrix
                with open(filepath, 'w') as f:
                    # Write standard parameters
                    yaml.dump(asdict(params), f, default_flow_style=False)
                    
                    # Add formatted rotation matrix representation
                    f.write("\n# Rotation matrix in readable format:\n")
                    f.write("# [R1 R2 R3]\n")
                    f.write("# [R4 R5 R6]\n")
                    f.write("# [R7 R8 R9]\n")
                    f.write("rotation_matrix_formatted: |\n")
                    for row in self.rotation:
                        f.write(f"  [{row[0]:11.8f} {row[1]:11.8f} {row[2]:11.8f}]\n")
                    
                    # Also add Euler angles for reference
                    f.write("\n# Euler angles (degrees) [Roll, Pitch, Yaw]:\n")
                    f.write("euler_angles_deg:\n")
                    f.write(f"  roll: {self.rot_vars['Roll'].get():.6f}\n")
                    f.write(f"  pitch: {self.rot_vars['Pitch'].get():.6f}\n")
                    f.write(f"  yaw: {self.rot_vars['Yaw'].get():.6f}\n")
                    
            messagebox.showinfo("Success", "Calibration saved successfully")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save calibration: {str(e)}")
    
    def load_calibration(self):
        """Load calibration parameters"""
        filepath = filedialog.askopenfilename(
            title="Load Calibration",
            filetypes=[("Calibration Files", "*.yaml;*.json"),
                      ("YAML", "*.yaml"),
                      ("JSON", "*.json")]
        )
        
        if not filepath:
            return
            
        try:
            with open(filepath, 'r') as f:
                if filepath.endswith('.json'):
                    data = json.load(f)
                else:
                    data = yaml.safe_load(f)
            
            self.rotation = np.array(data['rotation'])
            self.translation = np.array(data['translation'])
            self.camera_matrix = np.array(data['camera_matrix'])
            self.dist_coeffs = np.array(data.get('dist_coeffs', []))
            self.image_size = (data['image_width'], data['image_height'])
            
            # Update GUI
            for i, axis in enumerate(['X', 'Y', 'Z']):
                self.trans_vars[axis].set(self.translation[i])
        
            # Update rolling ball display
            self.draw_rolling_ball()
            
            # Convert rotation matrix to Euler angles
            sy = np.sqrt(self.rotation[0, 0]**2 + self.rotation[1, 0]**2)
            singular = sy < 1e-6
            
            if not singular:
                x = np.arctan2(self.rotation[2, 1], self.rotation[2, 2])
                y = np.arctan2(-self.rotation[2, 0], sy)
                z = np.arctan2(self.rotation[1, 0], self.rotation[0, 0])
            else:
                x = np.arctan2(-self.rotation[1, 2], self.rotation[1, 1])
                y = np.arctan2(-self.rotation[2, 0], sy)
                z = 0
            
            self.rot_vars['Roll'].set(np.rad2deg(x))
            self.rot_vars['Pitch'].set(np.rad2deg(y))
            self.rot_vars['Yaw'].set(np.rad2deg(z))
            
            self.update_display()
            messagebox.showinfo("Success", "Calibration loaded successfully")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load calibration: {str(e)}")
    
    def run(self):
        """Run the GUI"""
        # Schedule initial display update
        self.root.after(100, self.update_display)
        self.root.mainloop()


# Import PIL for image handling
try:
    from PIL import Image, ImageTk
except ImportError:
    print("Error: PIL/Pillow is required. Install with: pip install Pillow")
    sys.exit(1)


def main():
    """Main entry point"""
    print("Starting calibration GUI...")
    print(f"Available libraries: ROS1={HAS_ROS1}, ROS2={HAS_ROS2}, MCAP={HAS_MCAP}")
    
    # Check dependencies
    if not any([HAS_ROS1, HAS_ROS2, HAS_MCAP]):
        print("Error: No bag reading libraries found!")
        print("Install one of:")
        print("  - ROS1: pip install rosbag roslibpy")
        print("  - ROS2: pip install rosbag2-py")
        print("  - MCAP: pip install mcap mcap-ros1-support mcap-ros2-support")
        sys.exit(1)
    
    # Create and run GUI
    try:
        print("Creating CalibrationGUI instance...")
        app = CalibrationGUI()
        print("Running GUI...")
        app.run()
    except Exception as e:
        print(f"Error running GUI: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()