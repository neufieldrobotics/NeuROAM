# Ouster QoS Configuration Guide

## Current Settings

All payload configurations now use **System Default QoS** by default:
```yaml
use_system_default_qos: true
```

## QoS Profiles Explained

### Option 1: Sensor Data QoS (`use_system_default_qos: false`)
- **Reliability**: Best Effort
- **History**: Keep Last 5
- **Durability**: Volatile
- **Best for**: Real-time operation, streaming sensor data
- **Pros**: Lower latency, less bandwidth usage, better for lossy networks
- **Cons**: May drop messages under heavy load

### Option 2: System Default QoS (`use_system_default_qos: true`)
- **Reliability**: Reliable
- **History**: Keep Last 10
- **Durability**: Volatile
- **Best for**: ROS bag recording, guaranteed delivery
- **Pros**: No message loss, complete data capture
- **Cons**: Higher latency, more bandwidth, can cause network congestion

## When to Use Each Setting

### Use Sensor Data QoS (`false`) for:
✅ Real-time robot operation  
✅ Network bandwidth is limited  
✅ Latency matters more than completeness  
✅ Running over wireless networks (like your Doodle Labs setup)  
✅ Production deployments  

### Use System Default QoS (`true`) for:
✅ Recording rosbags for offline analysis  
✅ When you need guaranteed message delivery  
✅ Wired network connections with high bandwidth  
✅ Development/debugging when you need complete data  

## How to Change QoS Settings

### Method 1: Edit Config Files (Permanent)
Edit the payload-specific config files:
- `/src/ouster-ros/ouster-ros/config/driver_params_payload{0-4}.yaml`

Change the line:
```yaml
use_system_default_qos: false  # or true
```

Then restart the launch file.

### Method 2: Launch Argument Override (Temporary)
You can override at launch time by modifying `global_launch.py` to pass parameters:
```python
launch_arguments={
    'params_file': '...',
    # Add parameter overrides:
}.items()
```

### Method 3: Runtime Parameter Change (If Supported)
```bash
ros2 param set /ouster/os_driver use_system_default_qos false
```
Note: This requires restarting the node to take effect on publishers.

## Affected Topics

The QoS setting applies to all Ouster output topics:
- `/ouster/points` - Point cloud
- `/ouster/imu` - IMU data
- `/ouster/scan` - LaserScan
- `/ouster/signal_image`, `/ouster/nearir_image`, `/ouster/reflec_image` - Image topics
- `/ouster/range_image` - Range image

## Recommendations for NeuROAM

Given your multi-robot wireless mesh network setup with Doodle Labs:

**For Normal Operation:**
```yaml
use_system_default_qos: false  # Current setting - RECOMMENDED
```
This is optimal for your wireless network and real-time SLAM operations.

**For Data Collection Sessions:**
You may want to temporarily switch to:
```yaml
use_system_default_qos: true
```
But be aware this will significantly increase network traffic. Consider:
1. Using QoS override files for specific topics (see `custom_qos.yaml`)
2. Recording locally on each payload and offloading later
3. Compressing point clouds before transmission

## Related Configuration

See also:
- `/launch/custom_qos.yaml` - Topic-specific QoS overrides for rosbag recording
- Ouster driver docs: https://github.com/ouster-lidar/ouster-ros
