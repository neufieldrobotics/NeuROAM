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

- [1. (Before) Update the payload](#1-before-update-the-payload)
- [2. (Before) Check that all sensors are working](#2-before-check-that-all-sensors-are-working)
- [3. (Before) Check calibration](#3-before-check-calibration)
  - [3a. (Before) Gather calibration data](#3a-before-gather-calibration-data)
  - [3b. (Before) Process calibration data](#3b-before-process-calibration-data)
  - [3b. (Before) Check calibration data](#3b-before-check-calibration-data)
- [4. (During) Run the experiment](#4-during-run-the-experiment)
- [5. (After) Offload the data](#5-after-offload-the-data)
- [6. (After) Verify the data](#6-after-verify-the-data)


# 1. (Before) Update the payload

We should make sure that the payload is on the latest version of the software.
This will make sure that any fixes or improvements that have been made are
included on all payloads.

1. `cd ~/NeuROAM`
2. `git pull`
3. `git submodule update --init --recursive`
4. `colcon build --symlink-install`
5. `source install/setup.bash`
6.  TODO: Alan finish the rest of this. Run the sensors, make sure no crashes, check that all nodes
   are running, etc.


# 2. (Before) Check that all sensors are working

1. TODO: Launch all ROS nodes: (however we want to launch)
2. check that all nodes are running: `ros2 node list`
   1. TODO: update with node names
   2. ouster
   3. doodle-labs
   4. spinnaker
   5. vectornav
   6. ublox gps
   7. sensor monitor
3. check that all necessary topics are publishing:
   1. TODO: update with how we want to check topics
   2. ouster: `/os_cloud_node/points`
   3. doodle-labs: `/doodle_labs/scan`
   4. spinnaker: `/spinnaker/stereo/left/image_raw` and `/spinnaker/stereo/right/image_raw`
   5. vectornav: `/vectornav/ins`
   6. ublox gps: `/ublox_gps/fix`
   7. sensor monitor: `/sensor_monitor/status`


# 3. (Before) Check calibration

## 3a. (Before) Gather calibration data


## 3b. (Before) Process calibration data


## 3b. (Before) Check calibration data

# 4. (During) Run the experiment

1. Go to starting position
2. Open note-taking app - audio or text is fine. This will be used to take notes
   during the experiment.
3. TODO: Launch the recording script:
4. Check that the bag being recorded is named correctly:
   - The bag should be named `<payload>_<date>_<time>.bag`, where `<payload>` is the name of the payload
     (e.g., "payload1"), `<date>` is the date of the experiment in YYYYMMDD format, and
     `<time>` is the time of the experiment start in HHMM format.
   - Example: `payload1_20231001_1200.bag` for a payload named "payload1"
     starting an experiment on October 1, 2023 at 12:00.
5. Execute the experiment plan.
6. Take notes during the experiment! Note the approximate time that each event occurs.
   1. Anytime entering or exiting a building or floor
   2. Anything else of note: going up/down stairs or elevators, encountering another robot, etc.
   3. Any unexpected events: crashes, sensor failures, etc.
7. When the experiment is complete, stop the recording script.
8. Make sure to save the notes from the experiment. Save your notes file in the format
 `notes_<payload>_<date>_<time>.extension`, where <payload> is the name of the payload
   (e.g., "payload1"), <date> is the date of the experiment in YYYYMMDD format, and
   <time> is the time of the experiment start in HHMM format.

# 5. (After) Offload the data

1. Copy the data off of the payload onto the provided portable SSD
   1. The data should be copied to the directory `/data/<payload>/`, where `<payload>` is the name of the payload (e.g., "payload1")
2. Save the notes file in the same directory as the data.
   1. Make sure the notes file is named correctly: `notes_<payload>_<date>_<time>.<extension>`

# 6. (After) Verify the data

1. Use `ros2 bag info` to check that the bag file is complete and has all the expected topics.
2. Visualize the data using RViz/Foxglove Studio to ensure that the data looks correct.
   1. **Point cloud**: no gaps or weird artifacts
   2. **Stereo images**: images are clear, in focus, and similar colors/exposures
   3. **IMU data**: check that the IMU data is being published and looks reasonable
   4. **GPS data**: check that the GPS data is being published and looks reasonable
3. Check the notes file to ensure that all events are logged correctly and that the timestamps

TODO: add example of good and bad IMU data
