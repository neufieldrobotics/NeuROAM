# LiDAR-Camera Calibration GUI

A simple and efficient GUI tool for manual alignment of camera frames and LiDAR point clouds from ROS/MCAP files. This tool provides mouse-based manual alignment without complex algorithms, making it easy to calibrate your sensor setup.

## Features

- **Multi-format Support**: ROS1 (.bag), ROS2 (.db3), and MCAP (.mcap) files
- **Mouse-based Manual Alignment**: Intuitive drag-and-drop calibration
- **Real-time Visualization**: Live preview of point cloud overlay on camera images
- **No ROS Installation Required**: Works with fallback implementations
- **Export Calibration Parameters**: Save results in JSON or YAML format
- **Simple and Efficient**: Minimal dependencies and clean interface

## Installation

### Prerequisites

- Python 3.7+
- OpenCV
- NumPy
- Tkinter (usually included with Python)

### Quick Install

1. **Clone the repository:**
   ```bash
   git clone https://github.com/neufieldrobotics/NeuROAM/tree/lidracam
   git checkout lidracam
   cd lidracam_align
   ```

2. **Install dependencies:**
   ```bash
   pip install opencv-python numpy pillow
   
   # For MCAP support (recommended):
   pip install mcap mcap-ros1-support mcap-ros2-support
   
   # For enhanced ROS bag support:
   pip install bagpy
   ```

3. **Run the GUI:**
   ```bash
   python calibration_gui.py
   ```

### Conda Environment (Recommended)

```bash
# Create a new conda environment
conda create -n lidarcam python=3.10
conda activate lidarcam

# Install dependencies
pip install opencv-python numpy pillow mcap mcap-ros1-support mcap-ros2-support bagpy

# Run the tool
python calibration_gui.py
```

## Usage

### 1. Launch the GUI

![GUI Launch](demo/gui_1.png)

Run the calibration tool:
```bash
python calibration_gui.py
```

### 2. Load Your Data

![Load Data](demo/gui_2.png)

Click "Load Bag" and select your ROS bag file (.bag), ROS2 bag (.db3), or MCAP file (.mcap). The tool will automatically:
- **Detect the file format** (ROS1, ROS2, or MCAP)
- **Scan for available topics** (camera images, LiDAR point clouds, camera info)
- **Categorize topics** by type for easy selection

#### Automatic Topic Detection

The tool automatically detects and categorizes topics:
- **Camera topics**: Topics containing image data (e.g., `/cam_0/image_raw`, `/camera/image`)
- **LiDAR topics**: Topics containing point cloud data (e.g., `/ouster/points`, `/velodyne_points`)
- **Camera info topics**: Topics containing camera calibration data (e.g., `/cam_0/camera_info`)

#### Topic Selection Dialog

![Topic Selection](demo/gui_6.png)

If multiple camera topics are detected, you'll see a selection dialog allowing you to choose which camera to use for calibration. The tool will automatically match the corresponding camera info topic.

#### MCAP Files and Metadata

**Important**: For MCAP files, ensure that the `metadata.yaml` file is present in the same directory as your MCAP file. This file contains essential topic information that enables the tool to:
- Identify available topics without reading the entire file
- Handle potentially corrupted or incomplete MCAP files
- Provide faster topic discovery

Example directory structure:
```
data/
├── your_data.mcap
└── metadata.yaml
```

### 3. Manual Alignment

![Manual Alignment](demo/gui_3.png)

The calibration GUI provides multiple intuitive methods to align LiDAR points with camera images:

#### **Quick Start Alignment**

1. **Match Centers Button**: Click this first for an intelligent initial alignment
   - Automatically estimates the best starting position
   - Aligns LiDAR and camera coordinate centroids
   - Applies realistic initial rotation (camera looking slightly down)
   - Provides a good baseline for manual fine-tuning

2. **Auto Z-Align Button**: Use for ground plane alignment
   - Automatically detects the ground plane in LiDAR data
   - Aligns the ground plane with the camera's horizontal axis
   - Useful for automotive and outdoor scenarios

#### **Manual Alignment Controls**

**Mouse Controls (Rolling Ball Interface):**
- **Left click + drag**: Translate the point cloud in X/Y directions
- **Right click + drag**: Rotate the point cloud (intuitive rolling ball rotation)
- **Mouse wheel**: Zoom in/out for detailed alignment
- **Shift + mouse wheel**: Translate in Z direction (depth)

**Text Input Controls (Precise Values):**
- **Translation fields**: Enter exact X, Y, Z translation values in meters
- **Rotation fields**: Enter exact Roll, Pitch, Yaw angles in degrees
- **Real-time updates**: Changes apply immediately as you type
- **Range**: Translation ±10m, Rotation ±180°

**Slider Controls (Fine-tuning):**
- **Translation sliders**: Fine-tune X, Y, Z translation with visual feedback
- **Rotation sliders**: Adjust Roll, Pitch, Yaw angles smoothly
- **Point size slider**: Change visualization point size (1-10 pixels)
- **Smooth operation**: Continuous updates during slider movement

#### **Coordinate System Reference**

The tool displays coordinate system information to help with alignment:

**LiDAR Coordinate System:**
- X: Forward (red arrow)
- Y: Left (green arrow) 
- Z: Up (blue arrow)

**Camera Coordinate System:**
- X: Right
- Y: Down
- Z: Forward (into the scene)

#### **Alignment Strategy**

**Recommended workflow:**
1. **Start with "Match Centers"** - Gets you ~80% aligned
2. **Use mouse controls** for coarse adjustments
3. **Fine-tune with sliders** for precise alignment
4. **Use text inputs** for exact values if needed
5. **Apply "Auto Z-Align"** if ground plane alignment is important

**Visual Feedback:**
- Points are color-coded by depth (blue=close, red=far)
- Real-time projection shows alignment quality
- Point count indicator shows how many points are visible

### 4. Real-time Preview

![Real-time Preview](demo/gui_4.png)

See the alignment results in real-time as you adjust parameters. The LiDAR points are color-coded by depth for better visualization.


### 5. Dual-View Visualization

![Top-View Preview](demo/pc_top_view.png)

The GUI provides two synchronized views for comprehensive alignment verification:

#### **Camera View (Main Panel)**
- Shows LiDAR points projected onto the camera image
- Color-coded by depth (blue=close, red=far)
- Real-time updates during alignment adjustments
- Point count display shows alignment quality

#### **Top-View Panel (Bird's Eye)**
- Bird's eye view of the LiDAR point cloud
- Same color coding as camera view for consistency
- Helps understand 3D spatial relationships
- Useful for verifying translation and rotation in the horizontal plane
- Updates synchronously with camera view

**Benefits of Dual-View:**
- **Faster alignment**: See both perspectives simultaneously
- **Better spatial understanding**: Understand 3D transformations intuitively
- **Quality verification**: Cross-check alignment from multiple viewpoints
- **Easier debugging**: Identify alignment issues quickly

### 6. Save Calibration

![Save Results](demo/gui_5.png)

Once satisfied with the alignment, click "Save Calibration" to export your results in JSON or YAML format.

## Advanced Features

### **Transformation Matrix Output**

The tool displays the complete 4x4 transformation matrix in real-time:
```
Transformation matrix (LiDAR->Camera):
[[ 0.99862953  0.0516182   0.00863792  0.28929063]
 [ 0.          0.16504761 -0.9862856  -0.25641026]
 [-0.05233596  0.98493393  0.16482141  0.25641026]
 [ 0.          0.          0.          1.        ]]
```

This matrix represents the transformation from LiDAR coordinates to camera coordinates using the standard format:
- **Rotation (3x3)**: Upper-left block containing the rotation matrix
- **Translation (3x1)**: Right column containing X, Y, Z translation
- **Homogeneous**: Bottom row [0, 0, 0, 1] for proper matrix operations

### **Real-time Feedback**

The GUI provides continuous feedback during alignment:
- **Point count**: Shows how many LiDAR points are visible in the camera view
- **Coordinate bounds**: Displays the spatial extent of transformed points
- **Transformation details**: Live updates of rotation and translation parameters
- **Visual indicators**: Color-coded points and coordinate system references

### **Robust Point Processing**

The tool implements several filtering stages for optimal performance:
1. **Distance filtering**: Removes points beyond typical sensor ranges (0.5-100m)
2. **Behind-camera filtering**: Excludes points with negative Z coordinates
3. **Image bounds filtering**: Only shows points within the camera's field of view
4. **Outlier removal**: Filters extreme values that could skew alignment

## Output Format

The calibration tool outputs the following parameters:

### JSON Format
```json
{
  "rotation": [
    [0.99996192, -0.00872654, 0.00000000],
    [0.00872654, 0.99996192, 0.00000000],
    [0.00000000, 0.00000000, 1.00000000]
  ],
  "translation": [0.1, 0.05, 0.2],
  "camera_matrix": [
    [525.0, 0.0, 320.0],
    [0.0, 525.0, 240.0],
    [0.0, 0.0, 1.0]
  ],
  "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
  "image_width": 640,
  "image_height": 480
}
```

### YAML Format
```yaml
rotation:
- [0.99996192, -0.00872654, 0.00000000]
- [0.00872654, 0.99996192, 0.00000000]
- [0.00000000, 0.00000000, 1.00000000]
translation: [0.1, 0.05, 0.2]
camera_matrix:
- [525.0, 0.0, 320.0]
- [0.0, 525.0, 240.0]
- [0.0, 0.0, 1.0]
dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
image_width: 640
image_height: 480

# Rotation matrix in readable format:
# [R1 R2 R3]
# [R4 R5 R6]
# [R7 R8 R9]
rotation_matrix_formatted: |
  [ 0.99996192 -0.00872654  0.00000000]
  [ 0.00872654  0.99996192  0.00000000]
  [ 0.00000000  0.00000000  1.00000000]

# Euler angles (degrees) [Roll, Pitch, Yaw]:
euler_angles_deg:
  roll: 0.000000
  pitch: 0.000000
  yaw: 0.500000
```

## Supported Data Formats

| Format | Extension | Description | Requirements |
|--------|-----------|-------------|-------------|
| ROS1 Bag | `.bag` | Standard ROS1 bag files | None |
| ROS2 Bag | `.db3` | ROS2 SQLite database format | None |
| MCAP | `.mcap` | Modern container format for robotics data | Requires `metadata.yaml` in same directory |

### MCAP File Requirements

For MCAP files, the tool requires a `metadata.yaml` file in the same directory. This file should contain:
- Topic names and message types
- Message counts and timing information
- Bag file metadata

Example `metadata.yaml` structure:
```yaml
rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata:
        name: /cam_0/image_raw
        type: sensor_msgs/msg/Image
      message_count: 1000
    - topic_metadata:
        name: /ouster/points
        type: sensor_msgs/msg/PointCloud2
      message_count: 500
```

## Troubleshooting

### Common Issues

1. **"No bag reading libraries found" error:**
   ```bash
   pip install mcap mcap-ros1-support mcap-ros2-support
   ```

2. **"ROS messages not available" warning:**
   - This is normal and expected. The tool uses fallback implementations.
   - The GUI will still work perfectly.

3. **File not visible in dialog:**
   - Make sure your file has the correct extension (.bag, .db3, .mcap)
   - Try selecting "All files (*.*)" in the file dialog

4. **GUI closes immediately:**
   - Check the terminal for error messages
   - Ensure all dependencies are installed

5. **"No camera/LiDAR topics found" error:**
   - Check that your bag file contains the expected message types
   - For MCAP files, ensure `metadata.yaml` is present and correctly formatted
   - Verify topic names match expected patterns (e.g., topics containing "image", "camera", "lidar", "points")

6. **MCAP file reading issues:**
   - Ensure `metadata.yaml` file is in the same directory as the MCAP file
   - Check that the MCAP file is not corrupted or incomplete
   - Try using a different MCAP file or re-export your data

7. **Topic selection dialog doesn't appear:**
   - This is normal when only one camera topic is detected
   - The tool automatically selects the single available camera topic
   - Multiple camera topics will trigger the selection dialog

### Performance Tips

- For large datasets, consider downsampling your point clouds before calibration
- MCAP format generally provides better performance than ROS bags (when properly formatted)
- Close other applications to ensure smooth real-time visualization
- For MCAP files, having `metadata.yaml` significantly improves loading performance
- If you have multiple camera topics, pre-select the one you want to use to avoid the selection dialog

## Tips for Best Results

### **Alignment Quality Indicators**

Monitor these metrics for successful calibration:
- **Point count**: Aim for 1000+ visible points for good coverage
- **Spatial distribution**: Points should be spread across the image, not clustered
- **Depth variation**: Mix of close and far points (blue to red color range)
- **Edge alignment**: LiDAR points should align with object edges in the camera image

### **Common Alignment Scenarios**

**Automotive Setup (Forward-facing camera + LiDAR):**
1. Start with "Match Centers" 
2. Use "Auto Z-Align" for ground plane alignment
3. Fine-tune with small rotations (typically <10° in all axes)
4. Verify road/ground points align with image horizon

**Indoor/Handheld Setup:**
1. Use "Match Centers" for initial positioning
2. Focus on wall/object edge alignment
3. Use mouse controls for intuitive 3D rotation
4. Verify depth consistency across the scene

**Drone/Aerial Setup:**
1. "Match Centers" provides good starting point
2. Pay attention to pitch angle (camera looking down)
3. Use top-view to verify horizontal alignment
4. Check that ground features align properly

### **Troubleshooting Alignment Issues**

**No points visible after loading:**
- Check coordinate system conventions (LiDAR vs Camera)
- Try "Match Centers" to get initial positioning
- Verify your data contains synchronized timestamps
- Check that LiDAR and camera topics are from the same time period

**Points appear but don't align:**
- Use "Match Centers" first, then fine-tune manually
- Check if coordinate systems are consistent with your sensor setup
- Verify camera calibration parameters are correct
- Try "Auto Z-Align" if dealing with ground-based scenarios

**Alignment drifts during adjustment:**
- Use text inputs for precise values instead of sliders
- Make small incremental changes (±0.1m translation, ±5° rotation)
- Save intermediate results to avoid losing good alignments
- Use the dual-view to verify changes from multiple perspectives

## Sample Data

The repository includes sample data in the `lidar_camera_ros_data/` directory:
- `carla-v2.2-t10.bag` - Sample ROS bag file for testing

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.


**Note**: This tool is designed for manual calibration. For automated calibration algorithms, consider using specialized robotics calibration packages.
