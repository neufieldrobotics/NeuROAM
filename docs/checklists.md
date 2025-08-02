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

1. Launch all ROS nodes: `ros2 launch neuroam payload.launch.py`


# 3. (Before) Check calibration

## 3a. (Before) Gather calibration data


## 3b. (Before) Process calibration data


## 3b. (Before) Check calibration data