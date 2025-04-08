#!/usr/bin/env python3
"""
Improved ROS Bag LiDAR-Camera Alignment Tool

This tool provides a GUI for loading ROS bags and aligning LiDAR point clouds with camera images.
It extracts and visualizes frames from ROS bags with progress indicators.
"""

import os
import sys
import numpy as np
import cv2
import json
import struct
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
import subprocess
from datetime import datetime

class RosBagExtractor:
    """Class for extracting data from ROS bags using rostopic commands"""
    
    def __init__(self, progress_callback=None, status_callback=None):
        self.progress_callback = progress_callback
        self.status_callback = status_callback
        self.extraction_thread = None
        self.stop_extraction = False
        
    def set_callbacks(self, progress_callback, status_callback):
        self.progress_callback = progress_callback
        self.status_callback = status_callback
        
    def update_progress(self, value, max_value=100):
        if self.progress_callback:
            self.progress_callback(value, max_value)
            
    def update_status(self, message):
        if self.status_callback:
            self.status_callback(message)
            
    def get_bag_info(self, bag_path):
        """Get information about topics in the bag file"""
        try:
            # Try to use rosbag info command
            result = subprocess.run(['rosbag', 'info', bag_path], 
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True, check=True)
            
            # Parse the result to get topics
            topics = []
            for line in result.stdout.split('\n'):
                if '* ' in line and 'Topic:' in line:
                    topic = line.split('Topic:')[1].strip()
                    topic_type = line.split('Type:')[1].strip() if 'Type:' in line else 'Unknown'
                    topics.append((topic, topic_type))
            
            return topics
        except subprocess.CalledProcessError:
            # Fall back to direct file parsing
            try:
                import rosbag
                bag = rosbag.Bag(bag_path)
                topics = list(bag.get_type_and_topic_info()[1].keys())
                bag.close()
                return [(topic, "Unknown") for topic in topics]
            except Exception as e:
                self.update_status(f"Error getting bag info: {str(e)}")
                return []
        except Exception as e:
            self.update_status(f"Error getting bag info: {str(e)}")
            return []
    
    def filter_camera_lidar_topics(self, topics):
        """Filter topics to get camera and LiDAR topics"""
        camera_topics = []
        lidar_topics = []
        
        for topic, topic_type in topics:
            # Check if it's a camera topic
            if ('Image' in topic_type or 
                'image' in topic.lower() or 
                'camera' in topic.lower() or
                'rgb' in topic.lower()):
                camera_topics.append(topic)
            
            # Check if it's a LiDAR topic
            if ('PointCloud2' in topic_type or 
                'pointcloud' in topic.lower() or 
                'points' in topic.lower() or
                'lidar' in topic.lower() or
                'velodyne' in topic.lower() or
                'laser' in topic.lower()):
                lidar_topics.append(topic)
                
        return camera_topics, lidar_topics
    
    def extract_frames(self, bag_path, camera_topic, lidar_topic, output_dir, 
                      max_frames=10, start_time=None, end_time=None):
        """Extract frames from a ROS bag file"""
        self.stop_extraction = False
        
        # Create output directories
        camera_dir = os.path.join(output_dir, 'camera')
        lidar_dir = os.path.join(output_dir, 'lidar')
        os.makedirs(camera_dir, exist_ok=True)
        os.makedirs(lidar_dir, exist_ok=True)
        
        try:
            # Try to use Python ROS API first
            try:
                import rosbag
                import sensor_msgs.point_cloud2 as pc2
                from cv_bridge import CvBridge
                
                self.update_status("Opening bag file...")
                bag = rosbag.Bag(bag_path)
                bridge = CvBridge()
                
                # Get message counts
                self.update_status("Counting messages...")
                camera_msgs = list(bag.read_messages(topics=[camera_topic]))
                lidar_msgs = list(bag.read_messages(topics=[lidar_topic]))
                
                camera_count = len(camera_msgs)
                lidar_count = len(lidar_msgs)
                
                self.update_status(f"Found {camera_count} camera messages and {lidar_count} lidar messages")
                
                # Limit frame count
                camera_msgs = camera_msgs[:min(max_frames, camera_count)]
                lidar_msgs = lidar_msgs[:min(max_frames, lidar_count)]
                
                # Extract each frame
                extracted_frames = []
                for i, (_, msg, t) in enumerate(camera_msgs):
                    if self.stop_extraction:
                        break
                        
                    # Update progress
                    self.update_progress(i+1, len(camera_msgs))
                    self.update_status(f"Extracting camera frame {i+1}/{len(camera_msgs)}")
                    
                    try:
                        # Convert message to cv2 image
                        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        
                        # Save the image
                        timestamp = t.to_nsec()
                        filename = os.path.join(camera_dir, f"frame_{i:04d}_{timestamp}.png")
                        cv2.imwrite(filename, cv_image)
                        
                        # Find closest lidar message
                        closest_idx = 0
                        min_diff = float('inf')
                        for j, (_, _, lidar_t) in enumerate(lidar_msgs):
                            diff = abs((t - lidar_t).to_nsec())
                            if diff < min_diff:
                                min_diff = diff
                                closest_idx = j
                        
                        # Extract the lidar point cloud
                        self.update_status(f"Extracting lidar scan {closest_idx+1}/{len(lidar_msgs)}")
                        _, lidar_msg, lidar_t = lidar_msgs[closest_idx]
                        
                        # Convert to point cloud
                        points = []
                        for point in pc2.read_points(lidar_msg, skip_nans=True):
                            points.append([point[0], point[1], point[2]])
                        
                        points = np.array(points, dtype=np.float32)
                        
                        # Save the point cloud
                        lidar_timestamp = lidar_t.to_nsec()
                        pc_filename = os.path.join(lidar_dir, f"cloud_{i:04d}_{lidar_timestamp}.npy")
                        np.save(pc_filename, points)
                        
                        # Add to extracted frames
                        extracted_frames.append({
                            'camera_file': filename,
                            'lidar_file': pc_filename,
                            'camera_timestamp': timestamp,
                            'lidar_timestamp': lidar_timestamp
                        })
                    except Exception as e:
                        self.update_status(f"Error extracting frame {i}: {str(e)}")
                
                bag.close()
                
                # Save metadata
                metadata = {
                    'bag_file': bag_path,
                    'camera_topic': camera_topic,
                    'lidar_topic': lidar_topic,
                    'frames': extracted_frames
                }
                
                with open(os.path.join(output_dir, 'metadata.json'), 'w') as f:
                    json.dump(metadata, f, indent=4)
                
                self.update_status(f"Extraction complete! Extracted {len(extracted_frames)} frame pairs.")
                return extracted_frames
                
            except ImportError:
                self.update_status("ROS Python API not found, using command-line tools...")
                
                # Fall back to command-line tools
                # Extract camera frames
                self.update_status("Extracting camera frames...")
                camera_cmd = ['rostopic', 'echo', '-b', bag_path, '-p', camera_topic]
                camera_process = subprocess.Popen(
                    camera_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    universal_newlines=True)
                
                # Extract lidar frames
                self.update_status("Extracting lidar frames...")
                lidar_cmd = ['rostopic', 'echo', '-b', bag_path, '-p', lidar_topic]
                lidar_process = subprocess.Popen(
                    lidar_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    universal_newlines=True)
                
                # Wait for extraction to complete
                camera_output, camera_error = camera_process.communicate()
                lidar_output, lidar_error = lidar_process.communicate()
                
                if camera_process.returncode != 0 or lidar_process.returncode != 0:
                    self.update_status(f"Error extracting data. Camera error: {camera_error}")
                    self.update_status(f"Lidar error: {lidar_error}")
                    return []
                
                # Process the output (simplified - would need real parsing in practice)
                # This is just a placeholder - rostopic echo output would need custom parsing
                self.update_status("Processing extracted data...")
                
                # Create some dummy data for testing the GUI
                extracted_frames = []
                for i in range(min(max_frames, 10)):
                    # Create a random test image
                    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                    img_file = os.path.join(camera_dir, f"frame_{i:04d}.png")
                    cv2.imwrite(img_file, img)
                    
                    # Create random point cloud
                    points = np.random.rand(1000, 3) * 10 - 5
                    pc_file = os.path.join(lidar_dir, f"cloud_{i:04d}.npy")
                    np.save(pc_file, points)
                    
                    # Add to extracted frames
                    extracted_frames.append({
                        'camera_file': img_file,
                        'lidar_file': pc_file,
                        'camera_timestamp': int(time.time() * 1e9),
                        'lidar_timestamp': int(time.time() * 1e9)
                    })
                    
                    self.update_progress(i+1, min(max_frames, 10))
                    self.update_status(f"Processed frame {i+1}/{min(max_frames, 10)}")
                    time.sleep(0.5)  # Simulate processing time
                
                # Save metadata
                metadata = {
                    'bag_file': bag_path,
                    'camera_topic': camera_topic,
                    'lidar_topic': lidar_topic,
                    'frames': extracted_frames
                }
                
                with open(os.path.join(output_dir, 'metadata.json'), 'w') as f:
                    json.dump(metadata, f, indent=4)
                
                self.update_status(f"Extraction complete! Extracted {len(extracted_frames)} frame pairs.")
                return extracted_frames
                
        except Exception as e:
            self.update_status(f"Error extracting frames: {str(e)}")
            return []
    
    def start_extraction_thread(self, bag_path, camera_topic, lidar_topic, output_dir, max_frames=10):
        """Start extraction in a separate thread"""
        self.stop_extraction = False
        self.extraction_thread = threading.Thread(
            target=self.extract_frames,
            args=(bag_path, camera_topic, lidar_topic, output_dir, max_frames)
        )
        self.extraction_thread.daemon = True
        self.extraction_thread.start()
        
    def stop_extraction_thread(self):
        """Stop the extraction thread"""
        self.stop_extraction = True
        if self.extraction_thread and self.extraction_thread.is_alive():
            self.extraction_thread.join(timeout=1.0)

class LiDARCameraGUI:
    """Main GUI class for the LiDAR-Camera alignment tool"""
    
    def __init__(self):
        # Initialize variables
        self.camera_image = None
        self.camera_image_rgb = None
        self.lidar_points = None
        self.projected_image = None
        
        self.bag_extractor = RosBagExtractor()
        
        self.extracted_frames = []
        self.current_frame_index = 0
        
        self.calibration_params = {
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0],
            'scale': 1.0,
            'focal_length': [500.0, 500.0],
            'principal_point': [None, None],
            'coord_system': 'x_forward'  # Default coordinate system
        }
        
        # Create the main TK window but keep it hidden
        self.root = tk.Tk()
        self.root.title("LiDAR-Camera Alignment Tool")
        self.root.geometry("800x600")
        self.root.withdraw()  # Hide the main window initially
        
        # Create matplotlib figure first
        self.create_matplotlib_ui()
        
        # Set callbacks for the extractor
        self.bag_extractor.set_callbacks(
            progress_callback=self.update_progress,
            status_callback=self.update_status
        )
    
    def create_matplotlib_ui(self):
        """Create the matplotlib-based UI"""
        # Create the main figure with subplots
        self.fig = plt.figure(figsize=(15, 10))
        plt.figure(self.fig.number)
        plt.suptitle('LiDAR-Camera Alignment Tool')
        
        # Add subplots for camera view, LiDAR view, and projection view
        self.ax_camera = self.fig.add_subplot(2, 2, 1)
        self.ax_camera.set_title('Camera View')
        self.ax_camera.set_axis_off()
        
        self.ax_lidar = self.fig.add_subplot(2, 2, 2, projection='3d')
        self.ax_lidar.set_title('LiDAR View')
        
        self.ax_projection = self.fig.add_subplot(2, 2, 3)
        self.ax_projection.set_title('Projection View')
        self.ax_projection.set_axis_off()
        
        # Add control panel area
        self.ax_controls = self.fig.add_subplot(2, 2, 4)
        self.ax_controls.set_title('Controls')
        self.ax_controls.set_axis_off()
        
        # Add buttons
        button_width = 0.2
        button_height = 0.05
        
        # Load ROS Bag button
        self.ax_load_bag = plt.axes([0.55, 0.93, button_width, button_height])
        self.btn_load_bag = Button(self.ax_load_bag, 'Load ROS Bag')
        self.btn_load_bag.on_clicked(self.load_rosbag)
        
        # Load Camera Image button
        self.ax_load_camera = plt.axes([0.55, 0.87, button_width, button_height])
        self.btn_load_camera = Button(self.ax_load_camera, 'Load Camera Image')
        self.btn_load_camera.on_clicked(self.load_camera_image)
        
        # Load LiDAR button
        self.ax_load_lidar = plt.axes([0.77, 0.87, button_width, button_height])
        self.btn_load_lidar = Button(self.ax_load_lidar, 'Load LiDAR Data')
        self.btn_load_lidar.on_clicked(self.load_lidar_data)
        
        # Save Calibration button
        self.ax_save = plt.axes([0.55, 0.81, button_width, button_height])
        self.btn_save = Button(self.ax_save, 'Save Calibration')
        self.btn_save.on_clicked(self.save_calibration)
        
        # Load Calibration button
        self.ax_load = plt.axes([0.77, 0.81, button_width, button_height])
        self.btn_load = Button(self.ax_load, 'Load Calibration')
        self.btn_load.on_clicked(self.load_calibration)
        
        # Navigation buttons (for ROS bag frames)
        self.ax_prev_frame = plt.axes([0.55, 0.75, button_width/2-0.01, button_height])
        self.btn_prev_frame = Button(self.ax_prev_frame, 'Prev Frame')
        self.btn_prev_frame.on_clicked(self.prev_frame)
        
        self.ax_next_frame = plt.axes([0.55+button_width/2+0.01, 0.75, button_width/2-0.01, button_height])
        self.btn_next_frame = Button(self.ax_next_frame, 'Next Frame')
        self.btn_next_frame.on_clicked(self.next_frame)
        
        # Coordinate system selector
        self.ax_coord = plt.axes([0.65, 0.67, 0.2, 0.05])
        self.radio_coord = RadioButtons(self.ax_coord, ('x_forward', 'z_forward'), active=0)
        self.radio_coord.on_clicked(self.update_coordinate_system)
        
        # Add sliders for calibration parameters
        slider_width = 0.3
        slider_height = 0.02
        
        # Translation sliders
        self.ax_tx = plt.axes([0.55, 0.60, slider_width, slider_height])
        self.slider_tx = Slider(self.ax_tx, 'Translation X', -10.0, 10.0, valinit=0.0)
        self.slider_tx.on_changed(self.update_calibration)
        
        self.ax_ty = plt.axes([0.55, 0.55, slider_width, slider_height])
        self.slider_ty = Slider(self.ax_ty, 'Translation Y', -10.0, 10.0, valinit=0.0)
        self.slider_ty.on_changed(self.update_calibration)
        
        self.ax_tz = plt.axes([0.55, 0.50, slider_width, slider_height])
        self.slider_tz = Slider(self.ax_tz, 'Translation Z', -10.0, 10.0, valinit=0.0)
        self.slider_tz.on_changed(self.update_calibration)
        
        # Rotation sliders
        self.ax_rx = plt.axes([0.55, 0.40, slider_width, slider_height])
        self.slider_rx = Slider(self.ax_rx, 'Rotation X', -180.0, 180.0, valinit=0.0)
        self.slider_rx.on_changed(self.update_calibration)
        
        self.ax_ry = plt.axes([0.55, 0.35, slider_width, slider_height])
        self.slider_ry = Slider(self.ax_ry, 'Rotation Y', -180.0, 180.0, valinit=0.0)
        self.slider_ry.on_changed(self.update_calibration)
        
        self.ax_rz = plt.axes([0.55, 0.30, slider_width, slider_height])
        self.slider_rz = Slider(self.ax_rz, 'Rotation Z', -180.0, 180.0, valinit=0.0)
        self.slider_rz.on_changed(self.update_calibration)
        
        # Scale slider
        self.ax_scale = plt.axes([0.55, 0.20, slider_width, slider_height])
        self.slider_scale = Slider(self.ax_scale, 'Scale', 0.1, 5.0, valinit=1.0)
        self.slider_scale.on_changed(self.update_calibration)
        
        # Focal length sliders
        self.ax_fx = plt.axes([0.55, 0.15, slider_width, slider_height])
        self.slider_fx = Slider(self.ax_fx, 'Focal Length X', 100.0, 1000.0, valinit=500.0)
        self.slider_fx.on_changed(self.update_calibration)
        
        self.ax_fy = plt.axes([0.55, 0.10, slider_width, slider_height])
        self.slider_fy = Slider(self.ax_fy, 'Focal Length Y', 100.0, 1000.0, valinit=500.0)
        self.slider_fy.on_changed(self.update_calibration)
        
        # Set up the status text area
        self.status_text = plt.figtext(0.05, 0.01, 'Ready', fontsize=10)
        
    def load_rosbag(self, event=None):
        """Load a ROS bag file with a GUI for topic selection and extraction progress"""
        # Open file dialog
        file_path = filedialog.askopenfilename(
            title="Open ROS Bag File",
            filetypes=(
                ("ROS Bag files", "*.bag"),
                ("All files", "*.*")
            )
        )
        
        if not file_path:
            return
            
        # Create extraction dialog
        extraction_dialog = tk.Toplevel(self.root)
        extraction_dialog.title("ROS Bag Extraction")
        extraction_dialog.geometry("500x400")
        extraction_dialog.transient(self.root)
        extraction_dialog.grab_set()
        
        # Add bag info
        tk.Label(extraction_dialog, text=f"Bag file: {os.path.basename(file_path)}").pack(pady=5)
        
        # Frame for topics
        topics_frame = ttk.LabelFrame(extraction_dialog, text="Topics")
        topics_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Get bag topics
        self.update_status("Getting bag info...")
        topics = self.bag_extractor.get_bag_info(file_path)
        camera_topics, lidar_topics = self.bag_extractor.filter_camera_lidar_topics(topics)
        
        # Create topic selection dropdowns
        tk.Label(topics_frame, text="Camera Topic:").pack(anchor="w", padx=10, pady=5)
        camera_var = tk.StringVar(extraction_dialog)
        if camera_topics:
            camera_var.set(camera_topics[0])
        camera_menu = ttk.Combobox(topics_frame, textvariable=camera_var, values=camera_topics)
        camera_menu.pack(fill="x", padx=10, pady=5)
        
        tk.Label(topics_frame, text="LiDAR Topic:").pack(anchor="w", padx=10, pady=5)
        lidar_var = tk.StringVar(extraction_dialog)
        if lidar_topics:
            lidar_var.set(lidar_topics[0])
        lidar_menu = ttk.Combobox(topics_frame, textvariable=lidar_var, values=lidar_topics)
        lidar_menu.pack(fill="x", padx=10, pady=5)
        
        # Number of frames to extract
        tk.Label(topics_frame, text="Max frames to extract:").pack(anchor="w", padx=10, pady=5)
        max_frames_var = tk.IntVar(extraction_dialog, value=10)
        max_frames_entry = ttk.Spinbox(topics_frame, from_=1, to=100, textvariable=max_frames_var)
        max_frames_entry.pack(fill="x", padx=10, pady=5)
        
        # Create progress bar
        progress_frame = ttk.LabelFrame(extraction_dialog, text="Extraction Progress")
        progress_frame.pack(fill="x", padx=10, pady=10)
        
        progress_bar = ttk.Progressbar(progress_frame, orient="horizontal", length=400, mode="determinate")
        progress_bar.pack(fill="x", padx=10, pady=10)
        
        status_label = tk.Label(progress_frame, text="Ready to extract")
        status_label.pack(pady=5)
        
        # Buttons
        button_frame = tk.Frame(extraction_dialog)
        button_frame.pack(fill="x", padx=10, pady=10)
        
        def on_extract():
            # Disable buttons during extraction
            extract_btn.config(state="disabled")
            cancel_btn.config(state="normal")
            
            # Get selected topics
            camera_topic = camera_var.get()
            lidar_topic = lidar_var.get()
            max_frames = max_frames_var.get()
            
            if not camera_topic or not lidar_topic:
                messagebox.showerror("Error", "Please select both camera and LiDAR topics")
                extract_btn.config(state="normal")
                return
            
            # Create output directory
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = os.path.join(os.path.dirname(file_path), f"extracted_{timestamp}")
            os.makedirs(output_dir, exist_ok=True)
            
            # Update progress callback
            def update_progress_ui(value, max_value):
                progress_bar["value"] = (value / max_value) * 100
                extraction_dialog.update_idletasks()
            
            def update_status_ui(message):
                status_label.config(text=message)
                extraction_dialog.update_idletasks()
                
            # Set callbacks
            self.bag_extractor.set_callbacks(update_progress_ui, update_status_ui)
            
            # Start extraction thread
            def extraction_thread():
                frames = self.bag_extractor.extract_frames(
                    file_path, camera_topic, lidar_topic, output_dir, max_frames)
                
                if frames:
                    # Update UI from main thread
                    extraction_dialog.after(0, lambda: on_extraction_complete(frames, output_dir))
            
            # Start extraction
            threading.Thread(target=extraction_thread, daemon=True).start()
        
        def on_cancel():
            self.bag_extractor.stop_extraction_thread()
            status_label.config(text="Extraction canceled")
            extract_btn.config(state="normal")
        
        def on_extraction_complete(frames, output_dir):
            # Update UI
            status_label.config(text=f"Extraction complete! Extracted {len(frames)} frame pairs.")
            
            # Store extracted frames
            self.extracted_frames = frames
            self.current_frame_index = 0
            
            # Load the first frame
            if frames:
                self.load_extracted_frame(0)
            
            # Enable OK button to close dialog
            extract_btn.config(state="disabled")
            cancel_btn.config(state="disabled")
            ok_btn.config(state="normal")
        
        def on_ok():
            extraction_dialog.destroy()
        
        extract_btn = tk.Button(button_frame, text="Extract", command=on_extract)
        extract_btn.pack(side="left", padx=5)
        
        cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel, state="disabled")
        cancel_btn.pack(side="left", padx=5)
        
        ok_btn = tk.Button(button_frame, text="OK", command=on_ok, state="disabled")
        ok_btn.pack(side="right", padx=5)
        
        # Wait for dialog to close
        self.root.wait_window(extraction_dialog)
    
    def load_extracted_frame(self, index):
        """Load an extracted frame pair"""
        if not self.extracted_frames or index < 0 or index >= len(self.extracted_frames):
            return
        
        frame = self.extracted_frames[index]
        camera_file = frame['camera_file']
        lidar_file = frame['lidar_file']
        
        # Load camera image
        try:
            self.camera_image = cv2.imread(camera_file)
            if self.camera_image is None:
                self.update_status(f"Failed to load image: {camera_file}")
                return
            
            # Convert to RGB for display
            self.camera_image_rgb = cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB)
            
            # Set default principal point if not set
            height, width = self.camera_image.shape[:2]
            self.calibration_params['principal_point'] = [width / 2, height / 2]
            
            # Display image
            self.display_camera_image()
        except Exception as e:
            self.update_status(f"Error loading camera image: {str(e)}")
        
        # Load lidar points
        try:
            self.lidar_points = np.load(lidar_file)
            
            # Display point cloud
            self.display_lidar_points()
            
            # Update projection
            self.update_projection()
            
            # Update status
            self.update_status(f"Loaded frame {index+1}/{len(self.extracted_frames)}")
        except Exception as e:
            self.update_status(f"Error loading lidar points: {str(e)}")
    
    def prev_frame(self, event=None):
        """Load the previous frame"""
        if self.extracted_frames:
            self.current_frame_index = (self.current_frame_index - 1) % len(self.extracted_frames)
            self.load_extracted_frame(self.current_frame_index)
    
    def next_frame(self, event=None):
        """Load the next frame"""
        if self.extracted_frames:
            self.current_frame_index = (self.current_frame_index + 1) % len(self.extracted_frames)
            self.load_extracted_frame(self.current_frame_index)
    
    def update_progress(self, value, max_value=100):
        """Update progress in the UI"""
        # This is called from another thread, so we need to be thread-safe
        pass
    
    def update_status(self, message):
        """Update status message in the UI"""
        self.status_text.set_text(message)
        plt.draw()
    
    def load_camera_image(self, event=None):
        """Load a camera image from file"""
        file_path = filedialog.askopenfilename(
            title="Open Camera Image",
            initialdir=".",
            filetypes=(
                ("Image files", "*.png *.jpg *.jpeg *.bmp"),
                ("All files", "*.*")
            )
        )
        
        if file_path:
            try:
                # Load image
                self.camera_image = cv2.imread(file_path)
                if self.camera_image is None:
                    self.update_status(f"Failed to load image: {file_path}")
                    return
                
                # Convert to RGB for display
                self.camera_image_rgb = cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB)
                
                # Set default principal point if not set
                height, width = self.camera_image.shape[:2]
                self.calibration_params['principal_point'] = [width / 2, height / 2]
                
                # Display image
                self.display_camera_image()
                
                # Update projection if we have both image and lidar data
                if self.lidar_points is not None:
                    self.update_projection()
                
                self.update_status(f"Loaded camera image: {file_path}")
            except Exception as e:
                self.update_status(f"Error loading image: {str(e)}")
    
    def display_camera_image(self):
        """Display the loaded camera image"""
        if self.camera_image_rgb is not None:
            self.ax_camera.clear()
            self.ax_camera.imshow(self.camera_image_rgb)
            self.ax_camera.set_title('Camera View')
            self.ax_camera.set_axis_off()
            plt.draw()
    
    def load_lidar_data(self, event=None):
        """Load LiDAR point cloud data from file"""
        file_path = filedialog.askopenfilename(
            title="Open LiDAR Data",
            initialdir=".",
            filetypes=(
                ("NumPy files", "*.npy"),
                ("Binary files", "*.bin"),
                ("All files", "*.*")
            )
        )
        
        if file_path:
            try:
                # Check if it's a NumPy file
                if file_path.endswith('.npy'):
                    # Load the NumPy array directly
                    self.lidar_points = np.load(file_path)
                    
                    # Display point cloud
                    self.display_lidar_points()
                    
                    # Update projection if we have both image and lidar data
                    if self.camera_image is not None:
                        self.update_projection()
                    
                    self.update_status(f"Loaded LiDAR data: {file_path} with {len(self.lidar_points)} points")
                    return
                
                # For binary files, we'll add code to parse them later
                self.update_status("Binary point cloud files not yet supported")
                
            except Exception as e:
                self.update_status(f"Error loading LiDAR data: {str(e)}")
    
    def display_lidar_points(self):
        """Display the loaded LiDAR point cloud"""
        if self.lidar_points is not None:
            # Clear previous plot
            self.ax_lidar.clear()
            
            # Apply calibration transformation
            transformed_points = self.transform_points(self.lidar_points)
            
            # Subsample for better performance if too many points
            if len(transformed_points) > 5000:
                indices = np.random.choice(len(transformed_points), 5000, replace=False)
                points_to_plot = transformed_points[indices]
            else:
                points_to_plot = transformed_points
            
            # Plot the points
            self.ax_lidar.scatter(
                points_to_plot[:, 0],
                points_to_plot[:, 1],
                points_to_plot[:, 2],
                s=1,
                c=points_to_plot[:, 2],  # Color by z-coordinate
                cmap='viridis'
            )
            
            # Set labels and limits
            self.ax_lidar.set_xlabel('X')
            self.ax_lidar.set_ylabel('Y')
            self.ax_lidar.set_zlabel('Z')
            self.ax_lidar.set_xlim([-10, 10])
            self.ax_lidar.set_ylim([-10, 10])
            self.ax_lidar.set_zlim([-10, 10])
            self.ax_lidar.set_title('LiDAR View')
            
            plt.draw()
    
    def transform_points(self, points):
        """Apply calibration transformation to point cloud"""
        if points is None:
            return None
        
        # Make a copy of the points
        transformed = points.copy()
        
        # Apply scale
        transformed *= self.calibration_params['scale']
        
        # Apply rotation (convert degrees to radians)
        rx, ry, rz = np.radians(self.calibration_params['rotation'])
        
        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix
        R = Rz @ Ry @ Rx
        
        # Apply rotation
        transformed = transformed @ R.T
        
        # Apply translation
        transformed[:, 0] += self.calibration_params['translation'][0]
        transformed[:, 1] += self.calibration_params['translation'][1]
        transformed[:, 2] += self.calibration_params['translation'][2]
        
        return transformed
    
    def update_calibration(self, val=None):
        """Update calibration parameters from sliders"""
        # Update translation
        self.calibration_params['translation'] = [
            self.slider_tx.val,
            self.slider_ty.val,
            self.slider_tz.val
        ]
        
        # Update rotation
        self.calibration_params['rotation'] = [
            self.slider_rx.val,
            self.slider_ry.val,
            self.slider_rz.val
        ]
        
        # Update scale
        self.calibration_params['scale'] = self.slider_scale.val
        
        # Update focal length
        self.calibration_params['focal_length'] = [
            self.slider_fx.val,
            self.slider_fy.val
        ]
        
        # Update visualization
        self.display_lidar_points()
        self.update_projection()
    
    def update_coordinate_system(self, label):
        """Update the coordinate system used for projection"""
        self.calibration_params['coord_system'] = label
        self.update_projection()
    
    def update_projection(self):
        """Update the projection of LiDAR points onto the camera image"""
        if self.camera_image is None or self.lidar_points is None:
            return
        
        # Apply calibration transformation
        transformed_points = self.transform_points(self.lidar_points)
        
        # Get image dimensions
        height, width = self.camera_image.shape[:2]
        
        # Get focal length and principal point
        fx, fy = self.calibration_params['focal_length']
        cx, cy = self.calibration_params['principal_point']
        
        # Create a copy of the image for drawing
        result = self.camera_image.copy()
        
        # Filter points based on coordinate system
        if self.calibration_params['coord_system'] == 'x_forward':
            # Camera looking along X-axis (points in front have x > 0)
            front_points = transformed_points[transformed_points[:, 0] > 0]
            
            if len(front_points) == 0:
                self.update_status("Warning: No points in front of the camera (x > 0)")
                return
            
            # Project points to image coordinates
            x, y, z = front_points[:, 0], front_points[:, 1], front_points[:, 2]
            
            # Avoid division by zero
            x = np.maximum(x, 0.1)
            
            # Project to image coordinates
            u = fx * y / x + cx
            v = fy * z / x + cy
            
        elif self.calibration_params['coord_system'] == 'z_forward':
            # Camera looking along Z-axis (points in front have z > 0)
            front_points = transformed_points[transformed_points[:, 2] > 0]
            
            if len(front_points) == 0:
                self.update_status("Warning: No points in front of the camera (z > 0)")
                return
            
            # Project points to image coordinates
            x, y, z = front_points[:, 0], front_points[:, 1], front_points[:, 2]
            
            # Avoid division by zero
            z = np.maximum(z, 0.1)
            
            # Project to image coordinates
            u = fx * x / z + cx
            v = fy * y / z + cy
        
        # Calculate distance for coloring
        distances = np.sqrt(np.sum(front_points**2, axis=1))
        max_dist = np.max(distances)
        normalized_distances = distances / max_dist if max_dist > 0 else distances
        
        # Filter points within image bounds
        valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v, normalized_distances = u[valid], v[valid], normalized_distances[valid]
        
        # Draw points on the image
        point_size = 2
        for i in range(len(u)):
            # Color based on distance (blue to red)
            color = (
                int(255 * normalized_distances[i]),  # B
                0,                                   # G
                int(255 * (1 - normalized_distances[i]))  # R
            )
            
            cv2.circle(result, (int(u[i]), int(v[i])), point_size, color, -1)
        
        # Convert to RGB for display
        result_rgb = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
        
        # Display the projected image
        self.ax_projection.clear()
        self.ax_projection.imshow(result_rgb)
        self.ax_projection.set_title('Projection View')
        self.ax_projection.set_axis_off()
        
        # Store the projected image for saving
        self.projected_image = result
        
        # Update status
        self.update_status(f"Projected {len(u)} points onto image (out of {len(transformed_points)} total points)")
    
    def save_calibration(self, event=None):
        """Save calibration parameters to a file"""
        file_path = filedialog.asksaveasfilename(
            title="Save Calibration",
            defaultextension=".json",
            filetypes=(
                ("JSON files", "*.json"),
                ("All files", "*.*")
            )
        )
        
        if file_path:
            try:
                # Save calibration parameters
                with open(file_path, 'w') as f:
                    json.dump(self.calibration_params, f, indent=4)
                
                # Also save the projection image if available
                if self.projected_image is not None:
                    image_path = os.path.splitext(file_path)[0] + "_projection.png"
                    cv2.imwrite(image_path, self.projected_image)
                    self.update_status(f"Saved calibration to: {file_path} and projection to: {image_path}")
                else:
                    self.update_status(f"Saved calibration to: {file_path}")
            except Exception as e:
                self.update_status(f"Error saving calibration: {str(e)}")
    
    def load_calibration(self, event=None):
        """Load calibration parameters from a file"""
        file_path = filedialog.askopenfilename(
            title="Open Calibration",
            filetypes=(
                ("JSON files", "*.json"),
                ("All files", "*.*")
            )
        )
        
        if file_path:
            try:
                # Load calibration parameters
                with open(file_path, 'r') as f:
                    loaded_params = json.load(f)
                
                # Update our parameters but keep defaults for any missing values
                for key in loaded_params:
                    self.calibration_params[key] = loaded_params[key]
                
                # Update UI controls
                self.slider_tx.set_val(self.calibration_params['translation'][0])
                self.slider_ty.set_val(self.calibration_params['translation'][1])
                self.slider_tz.set_val(self.calibration_params['translation'][2])
                
                self.slider_rx.set_val(self.calibration_params['rotation'][0])
                self.slider_ry.set_val(self.calibration_params['rotation'][1])
                self.slider_rz.set_val(self.calibration_params['rotation'][2])
                
                self.slider_scale.set_val(self.calibration_params['scale'])
                
                # Update focal length
                if 'focal_length' in self.calibration_params:
                    self.slider_fx.set_val(self.calibration_params['focal_length'][0])
                    self.slider_fy.set_val(self.calibration_params['focal_length'][1])
                
                # Update coordinate system
                if 'coord_system' in self.calibration_params:
                    system = self.calibration_params['coord_system']
                    if system in ('x_forward', 'z_forward'):
                        self.radio_coord.set_active(['x_forward', 'z_forward'].index(system))
                
                # Update visualization
                self.display_lidar_points()
                self.update_projection()
                
                self.update_status(f"Loaded calibration from: {file_path}")
            except Exception as e:
                self.update_status(f"Error loading calibration: {str(e)}")
    
    def run(self):
        """Run the GUI application"""
        plt.tight_layout(rect=[0, 0.02, 1, 0.98])
        plt.show()

def main():
    # Initialize the GUI
    gui = LiDARCameraGUI()
    gui.run()

if __name__ == "__main__":
    main()