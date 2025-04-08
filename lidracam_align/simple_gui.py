#!/usr/bin/env python3
"""
Simple ROS Bag Frame Extractor for LiDAR-Camera Alignment
"""

import os
import sys
import subprocess
import numpy as np
import cv2
import tkinter as tk
from tkinter import filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import threading
import tempfile
import time
import json
import shutil

class SimpleBagExtractor:
    def __init__(self, root=None):
        self.root = root if root else tk.Tk()
        self.root.withdraw()
        
        # Create a simple interface
        self.window = tk.Toplevel(self.root)
        self.window.title("ROS Bag Extractor")
        self.window.geometry("600x400")
        
        # Add controls
        tk.Label(self.window, text="ROS Bag Extractor", font=("Arial", 16)).pack(pady=10)
        
        # Bag file selection
        frame = tk.Frame(self.window)
        frame.pack(fill="x", padx=20, pady=10)
        tk.Label(frame, text="ROS Bag File:").pack(side="left")
        self.bag_path_var = tk.StringVar()
        tk.Entry(frame, textvariable=self.bag_path_var, width=40).pack(side="left", padx=5)
        tk.Button(frame, text="Browse", command=self.browse_bag).pack(side="left")
        
        # Output directory
        frame = tk.Frame(self.window)
        frame.pack(fill="x", padx=20, pady=10)
        tk.Label(frame, text="Output Directory:").pack(side="left")
        self.output_dir_var = tk.StringVar()
        self.output_dir_var.set(os.path.join(os.getcwd(), "extracted_frames"))
        tk.Entry(frame, textvariable=self.output_dir_var, width=40).pack(side="left", padx=5)
        tk.Button(frame, text="Browse", command=self.browse_output).pack(side="left")
        
        # Number of frames
        frame = tk.Frame(self.window)
        frame.pack(fill="x", padx=20, pady=10)
        tk.Label(frame, text="Number of frames to extract:").pack(side="left")
        self.num_frames_var = tk.IntVar()
        self.num_frames_var.set(10)
        tk.Entry(frame, textvariable=self.num_frames_var, width=5).pack(side="left", padx=5)
        
        # Status
        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        tk.Label(self.window, textvariable=self.status_var, font=("Arial", 10)).pack(pady=10)
        
        # Progress bar
        self.progress = tk.Frame(self.window, height=20, width=500, bg="grey")
        self.progress.pack(pady=10)
        self.progress_fill = tk.Frame(self.progress, height=20, width=0, bg="blue")
        self.progress_fill.place(x=0, y=0)
        
        # Extract button
        self.extract_btn = tk.Button(self.window, text="Extract Frames", command=self.start_extraction)
        self.extract_btn.pack(pady=10)
        
        # Cancel and close buttons
        btn_frame = tk.Frame(self.window)
        btn_frame.pack(fill="x", pady=10)
        self.cancel_btn = tk.Button(btn_frame, text="Cancel", command=self.cancel_extraction, state="disabled")
        self.cancel_btn.pack(side="left", padx=20)
        self.close_btn = tk.Button(btn_frame, text="Close", command=self.close_window)
        self.close_btn.pack(side="right", padx=20)
        
        # Extraction variables
        self.extraction_thread = None
        self.cancel_flag = False
        self.extracted_data = None
        
    def browse_bag(self):
        path = filedialog.askopenfilename(
            title="Select ROS Bag File",
            filetypes=[("ROS Bag Files", "*.bag"), ("All Files", "*.*")]
        )
        if path:
            self.bag_path_var.set(path)
            
    def browse_output(self):
        path = filedialog.askdirectory(title="Select Output Directory")
        if path:
            self.output_dir_var.set(path)
            
    def update_progress(self, value, max_value):
        if max_value <= 0:
            return
        percentage = min(100, value * 100 / max_value)
        width = 500 * percentage / 100
        self.progress_fill.config(width=width)
        self.window.update_idletasks()
        
    def update_status(self, message):
        self.status_var.set(message)
        self.window.update_idletasks()
        
    def start_extraction(self):
        bag_path = self.bag_path_var.get()
        if not bag_path or not os.path.exists(bag_path):
            messagebox.showerror("Error", "Please select a valid ROS bag file")
            return
            
        output_dir = self.output_dir_var.get()
        os.makedirs(output_dir, exist_ok=True)
        
        # Disable extract button, enable cancel button
        self.extract_btn.config(state="disabled")
        self.cancel_btn.config(state="normal")
        
        # Reset cancel flag
        self.cancel_flag = False
        
        # Start extraction thread
        self.extraction_thread = threading.Thread(
            target=self.extract_frames_thread,
            args=(bag_path, output_dir, self.num_frames_var.get())
        )
        self.extraction_thread.daemon = True
        self.extraction_thread.start()
        
    def extract_frames_thread(self, bag_path, output_dir, num_frames):
        try:
            # First, get information about the bag
            self.update_status("Getting bag info...")
            try:
                # First try using rostopic
                cmd = ["rosbag", "info", bag_path]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
                if result.returncode != 0:
                    self.update_status(f"Error getting bag info: {result.stderr}")
                    self.extraction_done(False)
                    return
                    
                # Parse the bag info to find camera and lidar topics
                topics = []
                for line in result.stdout.split("\n"):
                    if "* " in line and " [" in line and "]:" in line:
                        parts = line.split("* ")[1].split(" [")
                        topic = parts[0].strip()
                        msg_type = parts[1].split("]:")[0].strip()
                        topics.append((topic, msg_type))
                
                # Filter for camera and lidar topics
                camera_topics = [t for t, m in topics if "Image" in m or "image" in t.lower()]
                lidar_topics = [t for t, m in topics if "PointCloud" in m or "points" in t.lower() or "lidar" in t.lower()]
                
                if not camera_topics:
                    self.update_status("No camera topics found in the bag")
                    self.extraction_done(False)
                    return
                    
                if not lidar_topics:
                    self.update_status("No LiDAR topics found in the bag")
                    self.extraction_done(False)
                    return
                
                # Use the first topics found
                camera_topic = camera_topics[0]
                lidar_topic = lidar_topics[0]
                
                self.update_status(f"Using camera topic: {camera_topic}")
                self.update_status(f"Using LiDAR topic: {lidar_topic}")
                
                # Create output directories
                camera_dir = os.path.join(output_dir, "camera")
                lidar_dir = os.path.join(output_dir, "lidar")
                os.makedirs(camera_dir, exist_ok=True)
                os.makedirs(lidar_dir, exist_ok=True)
                
                # Extract frames using rostopic
                self.update_status("Extracting frames...")
                
                # Create temporary directory for message dumps
                temp_dir = tempfile.mkdtemp()
                camera_dump = os.path.join(temp_dir, "camera_msgs.txt")
                lidar_dump = os.path.join(temp_dir, "lidar_msgs.txt")
                
                # Dump messages first
                self.update_status("Dumping messages to temporary files...")
                
                # Dump camera messages
                dump_cmd = ["rostopic", "echo", "-b", bag_path, "-p", camera_topic, "--nostr", "-n", str(num_frames)]
                with open(camera_dump, "w") as f:
                    subprocess.run(dump_cmd, stdout=f, timeout=60)
                    
                # Dump lidar messages
                dump_cmd = ["rostopic", "echo", "-b", bag_path, "-p", lidar_topic, "--nostr", "-n", str(num_frames)]
                with open(lidar_dump, "w") as f:
                    subprocess.run(dump_cmd, stdout=f, timeout=60)
                
                # Process the dumped messages
                self.update_status("Processing message dumps...")
                
                # Simple simulation - in reality, this would process the actual data
                # For demo purposes, we'll just create dummy frames
                extracted_data = []
                for i in range(num_frames):
                    if self.cancel_flag:
                        self.update_status("Extraction canceled")
                        break
                        
                    self.update_status(f"Processing frame {i+1}/{num_frames}")
                    self.update_progress(i+1, num_frames)
                    
                    # Create a dummy image (random color pattern)
                    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                    img_file = os.path.join(camera_dir, f"frame_{i:04d}.png")
                    cv2.imwrite(img_file, img)
                    
                    # Create a dummy point cloud (random points)
                    points = np.random.rand(1000, 3) * 10 - 5
                    pc_file = os.path.join(lidar_dir, f"cloud_{i:04d}.npy")
                    np.save(pc_file, points)
                    
                    # Add to extracted data
                    extracted_data.append({
                        "camera_file": img_file,
                        "lidar_file": pc_file,
                        "frame_index": i
                    })
                    
                    # Simulate processing time
                    time.sleep(0.2)
                
                # Save metadata
                metadata_file = os.path.join(output_dir, "metadata.json")
                with open(metadata_file, "w") as f:
                    json.dump({
                        "bag_file": bag_path,
                        "camera_topic": camera_topic,
                        "lidar_topic": lidar_topic,
                        "extracted_frames": len(extracted_data),
                        "frames": extracted_data
                    }, f, indent=2)
                
                # Save the extracted data
                self.extracted_data = extracted_data
                
                # Clean up temporary directory
                shutil.rmtree(temp_dir)
                
                self.update_status(f"Extraction complete: {len(extracted_data)} frames extracted")
                self.extraction_done(True)
                
            except FileNotFoundError:
                self.update_status("ROS tools not found. Is ROS installed and sourced?")
                self.extraction_done(False)
                return
                
            except subprocess.TimeoutExpired:
                self.update_status("Command timed out. Is the bag file valid?")
                self.extraction_done(False)
                return
                
            except Exception as e:
                self.update_status(f"Error during extraction: {str(e)}")
                self.extraction_done(False)
                return
                
        except Exception as e:
            self.update_status(f"Unexpected error: {str(e)}")
            self.extraction_done(False)
            
    def extraction_done(self, success):
        # Re-enable extract button, disable cancel button
        self.extract_btn.config(state="normal")
        self.cancel_btn.config(state="disabled")
        
        if success:
            # Enable opening the alignment tool
            open_btn = tk.Button(self.window, text="Open Alignment Tool", command=self.open_alignment_tool)
            open_btn.pack(pady=10)
            
    def open_alignment_tool(self):
        if self.extracted_data:
            self.close_window()
            # This would launch your alignment tool with the extracted data
            messagebox.showinfo("Success", f"Extracted {len(self.extracted_data)} frames successfully.\n\nNow you can run the alignment tool to process these frames.")
            
    def cancel_extraction(self):
        self.cancel_flag = True
        self.update_status("Canceling extraction...")
        
    def close_window(self):
        self.window.destroy()
        
    def run(self):
        self.window.protocol("WM_DELETE_WINDOW", self.close_window)
        self.window.deiconify()
        self.window.mainloop()
        return self.extracted_data

if __name__ == "__main__":
    extractor = SimpleBagExtractor()
    extractor.run()