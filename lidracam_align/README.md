# LiDAR-Camera Alignment Toolkit

This toolkit provides tools for extracting, converting, visualizing, and aligning LiDAR point clouds with camera images from ROS bag files.

## Overview

The LiDAR-Camera Alignment Toolkit consists of several components:

1. **Data Extraction**: Extract raw data from ROS bag files
2. **Data Conversion**: Convert raw binary data to usable formats
3. **Visualization**: Generate visualizations of camera images and LiDAR point clouds
4. **Alignment**: Tools for aligning LiDAR point clouds with camera images
5. **Calibration Explorer**: Explore different calibration parameters to find optimal alignment

## Components

### 1. Data Extraction (`extract_data.py`)

This script extracts camera images and LiDAR point clouds from a ROS bag file. It handles deserialization of ROS messages and saves the raw data to files.

Usage:
```bash
python extract_data.py --bag_file <path_to_bag_file> --output_dir <output_directory>
```

### 2. Data Conversion (`convert_raw_data.py`)

This script converts raw binary data extracted from ROS bag files into usable formats:
- Camera images are converted to PNG format
- LiDAR point clouds are converted to NumPy arrays

Usage:
```bash
python convert_raw_data.py
```

### 3. Visualization (`visualize_data.py`)

This script generates visualizations of the converted data:
- Camera images are displayed as-is
- LiDAR point clouds are visualized in 3D and as top-down views
- Simple overlays of LiDAR points on camera images are created

Usage:
```bash
python visualize_data.py
```

### 4. Alignment Tool (`align_tool.py`)

This command-line tool allows for manual alignment of LiDAR point clouds with camera images by adjusting calibration parameters.

Usage:
```bash
python align_tool.py --image <path_to_image> --points <path_to_points> --output <output_directory> [options]
```

Options:
- `--tx`, `--ty`, `--tz`: Translation parameters
- `--rx`, `--ry`, `--rz`: Rotation parameters (in degrees)
- `--scale`: Scale parameter
- `--fx`, `--fy`: Focal length parameters
- `--params`: Path to a JSON file with calibration parameters

### 5. Calibration Explorer (`calibration_explorer.py`)

This script generates a grid of visualizations with different parameter combinations to help find the optimal alignment.

Usage:
```bash
python calibration_explorer.py --image <path_to_image> --points <path_to_points> --output <output_directory> [options]
```

Options:
- `--tx-range`, `--ty-range`, `--tz-range`: Translation parameter ranges
- `--rx-range`, `--ry-range`, `--rz-range`: Rotation parameter ranges (in degrees)
- `--scale-range`: Scale parameter range
- `--fx-range`, `--fy-range`: Focal length parameter ranges

## GUI Application (`lidar_camera_gui.py`)

The GUI application provides a graphical interface for aligning LiDAR point clouds with camera images. It allows for interactive adjustment of calibration parameters.

**Note**: The GUI application may have issues in certain environments due to Qt dependencies. In such cases, use the command-line tools instead.

Usage:
```bash
python lidar_camera_gui.py
```

## Workflow

1. Extract data from ROS bag files using `extract_data.py`
2. Convert the raw data to usable formats using `convert_raw_data.py`
3. Visualize the data using `visualize_data.py`
4. Explore different calibration parameters using `calibration_explorer.py`
5. Fine-tune the alignment using `align_tool.py`
6. (Optional) Use the GUI application for interactive alignment

## Dependencies

- Python 3.6+
- NumPy
- OpenCV
- Matplotlib
- PyQt5 (for GUI application)
- rosbags (for ROS bag file handling)

Install dependencies:
```bash
pip install numpy opencv-python matplotlib pyqt5 rosbags
```

## Troubleshooting

### Qt Platform Plugin Issues

If you encounter Qt platform plugin issues when running the GUI application, try the following:

1. Use the command-line tools instead of the GUI application
2. Set the `QT_QPA_PLATFORM` environment variable:
   ```bash
   export QT_QPA_PLATFORM=offscreen
   ```
3. Install additional Qt dependencies:
   ```bash
   pip install pyqt5-tools
   ```

### Visualization Issues

If you encounter issues with matplotlib visualizations, try setting the backend to Agg:
```python
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
```
