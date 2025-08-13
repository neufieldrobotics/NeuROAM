# ROS bag recording tests

We are still dropping more messages than we would like. This document is a log
of the issues encountered and the results of different attempts to improve the
situation.

## Default Settings

As a baseline, here are the default settings and software used for recording:
- **ROS 2 Distro**: Humble
- **Recorder**: `ros2 bag record -s mcap -o ${record_bag_name} --max-cache-size 6442450944 --storage-preset-profile fastwrite`
- **DDS**: `rmw_zenoh`
- **Data-Heavy Topics**:
  - `/camera/left/image_raw`
  - `/camera/right/image_raw`
  - `/lidar/points`
- **QoS profile (on all topics)**:
  - Reliability: `RELIABLE`
  - History: `KEEP_LAST`
  - Depth: (varies by topic, typically 4-5)
  - Durability: `VOLATILE`
  - Lifespan: `Infinite`
  - Deadline: `Infinite`
  - Liveliness: `AUTOMATIC`
  - Liveliness Lease Duration: `Infinite`

## Attempted Improvements

| Attempt Name | Description                                                                   | Result                                                                         |
| ------- | ----------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| Use MCap       | Use `ros2 bag record` with `--storage mcap` and `--max-cache-size 6442450944` | Dropped messages, especially on high-rate topics like `/camera/left/image_raw` |
| 2       | Increase cache size to `107                                                   |