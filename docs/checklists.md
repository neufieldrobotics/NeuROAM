<!-- TODO (running list)
- close loop with all interested faculty (zhi tan, sarah o., dave rosen, michael everett, hanu, alireza)
- checklists
  - calibration
  - shake down
  - data upload
- experiment (dashboard)
  - who runs which robots
  - logs for the experiments
  - which payload goes on each robot
  - dataset naming convention and organization
- scripting
  - rosbag record should include <payload>_<date>_<time>
- timezones
  - make sure everything in GMT
- timestamped semantics
  - e.g., went through doorway, took elevator from EXP1 to EXP7
- calibration verification
  - given a fixed calibration, tries to visualize some data (e.g., stereo point
  cloud) using that calibration to make sure that it is good
- -->

This page covers all of the checklists that should be run through before, during,
and after an experiment. The checklists will be listed in the order that they
should be run through, so that anyone involved can scroll through the page
and check off the items as they are completed.

- [1. (Before) Update the payload software](#1-before-update-the-payload-software)
- [2. (Before) Check that all hardware is working](#2-before-check-that-all-hardware-is-working)
  - [2a. Power and mechanical checks](#2a-power-and-mechanical-checks)
  - [2b. Hardware-Software Checks](#2b-hardware-software-checks)
  - [2c. ROS Node Checks](#2c-ros-node-checks)
- [3. (Before) Check calibration](#3-before-check-calibration)
  - [3a. (Before) Gather calibration data](#3a-before-gather-calibration-data)
  - [3b. (Before) Process calibration data](#3b-before-process-calibration-data)
  - [3c. (Before) Check calibrations](#3c-before-check-calibrations)
  - [3d. (Before) Save calibration parameters](#3d-before-save-calibration-parameters)
- [4. (During) Run the experiment](#4-during-run-the-experiment)
  - [4a. Pre-experiment logging](#4a-pre-experiment-logging)
  - [4b. Experiment execution](#4b-experiment-execution)
- [5. (After) Offload the data](#5-after-offload-the-data)
- [6. (After) Reset the hardware](#6-after-reset-the-hardware)
- [7. (After) Verify the data](#7-after-verify-the-data)


# 1. (Before) Update the payload software

We should make sure that the payload is on the latest version of the software.
This will make sure that any fixes or improvements that have been made are
included on all payloads.

1. Make sure the right payload is on the right robot. See below table for payloads and robots.

| Payload Name | Robot Name    |
| ------------ | ------------- |
| payload0     | Spot          |
| payload1     | Go2W          |
| payload2     | Go2           |
| payload3     | AgileX Scout  |
| payload4     | AgileX Hunter |

2. Make sure the username, hostname, and password are correct for the payload.
   - Username: `neuroam`
   - Password: `neuroam`
   - Hostname: `payload0`, `payload1`, `payload2`, `payload3`, or `payload4` depending on the payload.
3. `cd ~/NeuROAM`
4. `git pull`
5. `git submodule update --init --recursive`
6. `colcon build --symlink-install`
7. `source install/setup.bash`

# 2. (Before) Check that all hardware is working

## 2a. Power and mechanical checks

1. Check that the payload is securely attached to the robot.
2. Check cables and connections: all cables should be securely connected and not damaged.
3. Check batteries: both the payload and the robot should be fully charged.
4. Camera lenses should be clean and unobstructed. The cap should be removed from the stereo cameras.

## 2b. Hardware-Software Checks

1. Disk space: run `df -h` to check that there is enough disk space on the payload.
   - The harddrive is probably under `/dev/sda1` or `/dev/nvme0n1p1`.
   - We want at least `1TB` of space under `Available` for the experiment.
   - If there is not enough space, please offload data from previous experiments to the storage server.
2. TODO: chrony/ptp status
3. Connectivity checks: try to ping every robot. You can use the script `ping_robots.sh` to do this.
   - If a robot is not reachable, check the network connection and make sure the robot is powered on.
   - If you're looking for a specific robot, you can also independently use the `ping <ip_address>` command to check connectivity with a single robot. See the table below for the IP addresses of each payload.

<!-- table to say payload to IP mapping for ping tests -->

| Payload Name | IP Address   |
| ------------ | ------------ |
| payload0     | 10.19.30.100 |
| payload1     | 10.19.30.101 |
| payload2     | 10.19.30.102 |
| payload3     | 10.19.30.103 |
| payload4     | 10.19.30.104 |


## 2c. ROS Node Checks

1. TODO: ROS_DOMAIN_ID: Make sure that the ROS_DOMAIN_ID is set correctly for the payload. See below for the
   payloads and their corresponding ROS_DOMAIN_IDs.

| Payload Name | ROS_DOMAIN_ID |
| ------------ | ------------- |
| payload0     | 0             |
| payload1     | 1             |
| payload2     | 2             |
| payload3     | 3             |
| payload4     | 4             |

2. TODO: Launch all ROS nodes: (however we want to launch)
3. check that all nodes are running: `ros2 node list`
   1. TODO: update with node names
   2. ouster
   3. doodle-labs
   4. spinnaker
   5. vectornav
   6. ublox gps
   7. sensor monitor
4. check that all necessary topics are publishing: `ros2 topic list`
   1. TODO: update with how we want to check topics
   2. ouster: `/os_cloud_node/points`
   3. doodle-labs: `/doodle_labs/scan`
   4. spinnaker: `/spinnaker/stereo/left/image_raw` and `/spinnaker/stereo/right/image_raw`
   5. vectornav: `/vectornav/ins`
   6. ublox gps: `/ublox_gps/fix`
   7. sensor monitor: `/sensor_monitor/status`

# 3. (Before) Check calibration

## 3a. (Before) Gather calibration data

TODO: Describe how to gather calibration data for each sensor.

## 3b. (Before) Process calibration data

TODO: Describe how to process calibration data for each sensor.

## 3c. (Before) Check calibrations

TODO: Finish the calibration checks

This may get more sophisticated in the future. For now, we will check that the
estimated calibration parameters are within a reasonable range. This will ensure
that the calibration is not wildly off, but we should try to make this scheme a
little more robust in the future.

Make sure that the calibration parameters are within the following ranges:
1. TODO LIDAR to IMU transformation:
  - x: [-0.5, 0.5] m
  - y: [-0.5, 0.5] m
  - z: [-0.5, 0.5] m
  - rotation:
2. TODO Stereo camera to IMU transformation:
  - x: [-0.5, 0.5] m
  - y: [-0.5, 0.5] m
  - z: [-0.5, 0.5] m
  - rotation:
3. TODO Stereo camera params:
  - fx: [100, 1000] px
  - fy: [100, 1000] px
  - cx: [0, 1920] px
  - cy: [0, 1080] px
  - baseline: [0.05, 0.5] m
4. TODO Vectornav IMU calibration:
  - accelerometer bias: [-0.1, 0.1] m/s^2
  - gyroscope bias: [-0.1, 0.1] rad/s
5. TODO: timing offsets
6. TODO: visualize point clouds from lidar and stereo cameras to ensure that they are aligned
   - This can be done using RViz or Foxglove Studio.
   - Check that the point clouds are aligned and that there are no large gaps or misalignments.

## 3d. (Before) Save calibration parameters

TODO: where do we want to save the calibration data?

All calibration parameters should be saved to a date-stamped directory in the
payload's data directory. The directory should be `~/data/calibration/<date>/`,
where `<date>` is the date of the calibration in YYYYMMDD format.

The calibration files should have timestamped names in the format
`<calibration_name>_<payload>_<date>_<time>.yaml`, where `<payload>` is the name of the payload
(e.g., "payload1"), `<date>` is the date of the calibration in YYYYMMDD format, and
`<time>` is the time of the calibration in HHMM format.

1. Save the LIDAR to IMU transformation parameters in the file
    `lidar_imu_calibration_<payload>_<date>_<time>.yaml`
2. Save the stereo camera to IMU transformation parameters in the file
    `stereo_imu_calibration_<payload>_<date>_<time>.yaml`
3. Save the stereo camera parameters in the file
    `stereo_camera_calibration_<payload>_<date>_<time>.yaml`
4. Save the Vectornav IMU calibration parameters in the file
    `vectornav_imu_calibration_<payload>_<date>_<time>.yaml`
5. TODO: save other calibration parameters as needed.

# 4. (During) Run the experiment

## 4a. Pre-experiment logging

1. Take the following notes before starting the experiment:
   - Date and time of the experiment
   - Weather and lighting
     - e.g., sunny, cloudy, rainy, etc.
     - Temperature
   - Payload name
   - Robot name
   - Operator name
   - Any special instructions or considerations for the experiment

## 4b. Experiment execution

1. Make sure all other robots are ready
2. Go to starting position
3. Open note-taking app - audio or text is fine. This will be used to take notes
   during the experiment.
4. TODO: Launch the recording script:
5. Check that the bag being recorded is named correctly:
   - The bag should be named `<payload>_<date>_<time>.bag`, where `<payload>` is the name of the payload
     (e.g., "payload1"), `<date>` is the date of the experiment in YYYYMMDD format, and
     `<time>` is the time of the experiment start in HHMM format.
   - Example: `payload1_20231001_1200.bag` for a payload named "payload1"
     starting an experiment on October 1, 2023 at 12:00.
6. Execute the experiment plan.
7. Take notes during the experiment! Note the approximate time that each event occurs.
   1. Anytime entering or exiting a building or floor
   2. Anything else of note: going up/down stairs or elevators, encountering another robot, etc.
   3. Any unexpected events: crashes, sensor failures, etc.
8. When the experiment is complete, stop the recording script.

9. Save the notes from the experiment. Save your notes file in the format
 `notes_<payload>_<date>_<time>.extension`, where <payload> is the name of the payload
   (e.g., "payload1"), <date> is the date of the experiment in YYYYMMDD format, and
   <time> is the time of the experiment start in HHMM format.


# 5. (After) Offload the data

1. Copy the data off of the payload onto the provided portable SSD
   1. The data should be copied to the directory `/data/<payload>/`, where `<payload>` is the name of the payload (e.g., "payload1")
2. Save the notes file in the same directory as the data.
   1. Make sure the notes file is named correctly: `notes_<payload>_<date>_<time>.<extension>`
3. TODO: Using the SSD move the data to the storage server.
   - TODO: The data should be moved to the directory `/data/<payload>/<date>/`,
4. TODO: Use `checksum` to verify that the data was copied correctly.


# 6. (After) Reset the hardware

1. Power off the payload and robot.
2. Charge the batteries: both the payload and the robot should be set to fully charge.
3. TODO: clean lenses?

# 7. (After) Verify the data

1. Use `ros2 bag info` to check that the bag file is complete and has all the expected topics.
   1. TODO: update with expected topics
   - ouster: `/os_cloud_node/points`
   - doodle-labs: `/doodle_labs/scan`
   - spinnaker: `/spinnaker/stereo/left/image_raw` and `/spinnaker/stereo/right/image_raw`
   - vectornav: `/vectornav/ins`
   - ublox gps: `/ublox_gps/fix`
   - sensor monitor: `/sensor_monitor/status`
2. Visualize the data using RViz/Foxglove Studio to ensure that the data looks correct.
   1. TODO: Using the `foxglove studio` configuration saved in the `docs/foxglove` directory, open the bag file and check the following.
   2. **Point cloud**: no gaps or weird artifacts
   3. **Stereo images**: images are clear, in focus, and similar colors/exposures
   4. **IMU data**: check that the IMU data is being published and looks reasonable
3. Check the notes file to ensure that all events are logged correctly and that the timestamps
4. Once the ROS bag and notes are copied, delete the bag file from the payload to free up space.

TODO: add example of good and bad IMU data
TODO: add example of good and bag images
