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
from pathlib import Path
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional, Tuple, List
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image as PILImage, ImageTk
# Handle different bag formats
try:
    import rosbag
    HAS_ROS1 = True
except ImportError:
    try:
        import bagpy
        HAS_ROS1 = True
        HAS_BAGPY = True
    except ImportError:
        HAS_ROS1 = False
        HAS_BAGPY = False

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

try:
    from mcap.reader import make_reader
    from mcap_ros1.reader import DecoderFactory as Ros1DecoderFactory
    from mcap_ros2.reader import DecoderFactory as Ros2DecoderFactory
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
            if hasattr(img_msg, 'data'):
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
        with open(self.filepath, "rb") as f:
            reader = make_reader(f)
            
            # Detect ROS version from schema
            decoder_factory = None
            for schema in reader.get_summary().schemas.values():
                if "ros1" in schema.name:
                    decoder_factory = Ros1DecoderFactory()
                    break
                elif "ros2" in schema.name:
                    decoder_factory = Ros2DecoderFactory()
                    break
            
            if not decoder_factory:
                # Default to ROS1 if can't detect
                decoder_factory = Ros1DecoderFactory()
            
            # Read messages
            for schema, channel, message in reader.iter_messages(
                    topics=topics,
                    decoder_factories=[decoder_factory]):
                
                topic = channel.topic
                timestamp = message.publish_time * 1e-9  # nanoseconds to seconds
                
                # Decode message
                decoded_msg = decoder_factory.decoder_for(schema.name, schema.data)(message.data)
                yield topic, decoded_msg, timestamp
    
    def close(self):
        pass  # File closes automatically with context manager

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
        for i, axis in enumerate(['X', 'Y', 'Z']):
            frame = ttk.Frame(control_frame)
            frame.pack(fill=tk.X, padx=10, pady=2)
            ttk.Label(frame, text=f"{axis}:", width=3).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.trans_vars[axis] = var
            scale = ttk.Scale(frame, from_=-5.0, to=5.0, variable=var, 
                            command=lambda v, a=i: self.update_translation(a, float(v)))
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ttk.Label(frame, textvariable=var, width=8).pack(side=tk.LEFT)
        
        # Rotation controls
        ttk.Label(control_frame, text="Rotation (degrees):", font=('Arial', 10, 'bold')).pack(pady=(20, 5))
        
        self.rot_vars = {}
        for i, axis in enumerate(['Roll', 'Pitch', 'Yaw']):
            frame = ttk.Frame(control_frame)
            frame.pack(fill=tk.X, padx=10, pady=2)
            ttk.Label(frame, text=f"{axis}:", width=5).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.rot_vars[axis] = var
            scale = ttk.Scale(frame, from_=-180, to=180, variable=var,
                            command=lambda v, a=i: self.update_rotation(a, float(v)))
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ttk.Label(frame, textvariable=var, width=8).pack(side=tk.LEFT)
        
        # Frame navigation
        ttk.Label(control_frame, text="Frame Navigation:", font=('Arial', 10, 'bold')).pack(pady=(20, 5))
        nav_frame = ttk.Frame(control_frame)
        nav_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(nav_frame, text="<<", command=self.prev_frame).pack(side=tk.LEFT, padx=2)
        self.frame_label = ttk.Label(nav_frame, text="0/0")
        self.frame_label.pack(side=tk.LEFT, padx=10)
        ttk.Button(nav_frame, text=">>", command=self.next_frame).pack(side=tk.LEFT, padx=2)
        
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
        
        # Right panel - Display
        self.canvas_frame = ttk.Frame(main_frame)
        self.canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # Create canvas for image display
        self.canvas = tk.Canvas(self.canvas_frame, bg='black')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
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
        
    def load_bag(self):
        """Load ROS bag or MCAP file"""
        filepath = filedialog.askopenfilename(
            title="Select ROS bag or MCAP file",
            filetypes=[("ROS1 Bag", "*.bag"),
                      ("ROS2 Bag", "*.db3"),
                      ("MCAP", "*.mcap"),
                      ("All files", "*.*")]
        )
        
        if not filepath:
            return
            
        try:
            # Clear existing data
            self.frames = []
            self.current_frame = 0
            
            # Read bag file
            reader = BagReader.open_bag(filepath)
            
            # Topics to read
            camera_topic = "/carla/ego_vehicle/rgb_front/image"
            lidar_topic = "/carla/ego_vehicle/lidar"
            info_topic = "/carla/ego_vehicle/rgb_front/camera_info"
            
            # Read messages
            messages = {}
            for topic, msg, t in reader.read_messages([camera_topic, lidar_topic, info_topic]):
                if topic not in messages:
                    messages[topic] = []
                messages[topic].append((msg, t))
            
            # Extract camera info
            if info_topic in messages and messages[info_topic]:
                info_msg = messages[info_topic][0][0]
                self.camera_matrix = np.array(info_msg.K).reshape(3, 3)
                self.dist_coeffs = np.array(info_msg.D)
                self.image_size = (info_msg.width, info_msg.height)
            
            # Synchronize frames
            if camera_topic in messages and lidar_topic in messages:
                camera_msgs = messages[camera_topic]
                lidar_msgs = messages[lidar_topic]
                
                # Simple synchronization - pair closest timestamps
                for cam_msg, cam_t in camera_msgs[:20]:  # Limit frames for performance
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
        for point in pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)
    
    def project_points(self, points_3d):
        """Project 3D points to 2D image plane"""
        if self.camera_matrix is None or len(points_3d) == 0:
            return np.array([])
            
        # Apply extrinsic transformation
        points_transformed = (self.rotation @ points_3d.T).T + self.translation
        
        # Remove points behind camera
        valid_mask = points_transformed[:, 2] > 0.1
        points_transformed = points_transformed[valid_mask]
        
        if len(points_transformed) == 0:
            return np.array([])
        
        # Project to image plane
        points_2d = self.camera_matrix @ points_transformed.T
        points_2d = points_2d[:2] / points_2d[2]
        points_2d = points_2d.T
        
        # Filter points within image bounds
        valid_mask = (points_2d[:, 0] >= 0) & (points_2d[:, 0] < self.image_size[0]) & \
                    (points_2d[:, 1] >= 0) & (points_2d[:, 1] < self.image_size[1])
        
        # Add depth for coloring
        depths = points_transformed[valid_mask, 2]
        points_2d = points_2d[valid_mask]
        
        return np.column_stack([points_2d, depths])
    
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
    
    def update_translation(self, axis, value):
        """Update translation from GUI"""
        self.translation[axis] = value
        self.update_display()
    
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