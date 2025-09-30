# Documentation to Debayer Camera topics and Filter Bags

Process ROS2 bags with debayering, sensor filtering, and ROS1 conversion.

## Installation

```bash
# Install dependencies
pip3 install opencv-python numpy pyyaml
pip3 install rosbag rospkg  # For ROS1 conversion

# Source ROS2
source /opt/ros/humble/setup.bash
```

## Files Required

- debayer_and_filter_bags.py

- sensor_topics.yaml (for custom topic filtering)

## Usage

### Basic Commands

```bash
# Check available commands and arguments
python3 debayer_and_filter_bags.py --help

# Process all topics (no changes)
python3 debayer_and_filter_bags.py /path/to/bag

# Enable debayering for all cameras (saves both compressed and debayered camera topics)
python3 debayer_and_filter_bags.py /path/to/bag --debayer

# Convert to ROS1
python3 debayer_and_filter_bags.py /path/to/bag --convert-ros1
```

### Sensor Filtering

```bash
# LiDAR + IMU only
python3 debayer_and_filter_bags.py /path/to/bag --lidar --imu

# Camera only (automatically debayers and replaces compressed)
python3 debayer_and_filter_bags.py /path/to/bag --camera

# Camera + IMU (for VIO)
python3 debayer_and_filter_bags.py /path/to/bag --camera --imu

# GPS + IMU + Lidar
python3 debayer_and_filter_bags.py /path/to/bag --gps --imu --lidar
```

### Options

```bash
--output-bag PATH    # Specify output location
--force, -f          # Overwrite existing output
--in-place           # Modify original bag
--convert-ros1       # Convert to ROS1 after processing
--ros1-output PATH   # ROS1 output path
--debayer            # Enable debayering (for all topics mode)
--config PATH        # Custom config file
--cam0-topic TOPIC   # Custom cam0 compressed topic
--cam1-topic TOPIC   # Custom cam1 compressed topic
```

## Behavior

### Camera Group
When `--camera` is used:
- Automatically enables debayering
- Replaces compressed topics with debayered RGB images
- Excludes compressed topics from output
- Result: Uncompressed RGB images only

### Debayer Flag
When `--debayer` is used alone (without sensor groups):
- Adds debayered topics alongside compressed
- Keeps all original topics
- Result: Both compressed and debayered images

### Topic Filtering
When sensor groups are specified:
- Only includes topics for selected sensors
- Always includes system topics (tf, tf_static, etc.)
- Significantly reduces bag size

## Output Naming

- Default: `<input_bag>_processed` (with debayering)
- Default: `<input_bag>_filtered` (without debayering)
- Custom: Use `--output-bag`
- Auto-increment: `_1`, `_2` if output exists


## Notes

- Won't double-debayer if bag already has debayered topics
- Camera group always debayers and replaces compressed
- Use `--force` to overwrite existing outputs
- Processes sequentially to minimize memory usage
