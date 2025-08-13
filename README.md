# NeuROAM

This document outlines the steps to set up the ROS 2 Humble workspace containing the drivers for the sensors used in the NeuROAM project, managed via Git submodules.

## 1. Prerequisites
* **Jetson:** Flashed with Jetpack 6.1
* **Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS Version:** ROS 2 Humble Hawksbill ([rosHumble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) installation)
* **Git:** Required for cloning and submodules (`sudo apt install git`)
* **FLIR Spinnaker SDK:**
  * **Required:** Download manually from the FLIR website ([SpinnerSDK](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis) for Linux Ubuntu 22.04, ARM64)
  * During installation accept the following:
    * Add your user to the `flirimaging` group
    * Set USB FS memory to 1000MB
    * Add Examples to PATH and register GenTL producer
    * `sudo reboot`
  * Verify installation by using `SpinView` tool
* **System Dependencies:**
  ```bash
    sudo apt update
    sudo apt install python3-rosdep libudev-dev \
         libjsoncpp-dev libeigen3-dev libtins-dev libpcap-dev \
         flatbuffers-compiler libflatbuffers-dev libglfw3-dev \
         python3-colcon-common-extensions python3-colcon-argcomplete \
         ros-humble-image-transport-plugins
    ```
* **User Groups:**
    ```bash
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G flirimaging $USER
    sudo reboot
    ```
## 2. Workspace Setup
**Clone the `NeuROAM` Repo:**
   ```bash
   cd ~
   git clone https://github.com/neufieldrobotics/NeuROAM.git NeuROAM
   cd ~/NeuROAM
   git checkout Drivers
   git submodule update --init --recursive
   ```
**Install ROS Dependencies:**
   ```bash
   cd ~/NeuROAM
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install # Takes about 10 minutes
   source install/setup.bash
   ```
## 3. Hardware Setup & Configuration
* **Connections:** Connect all USBs and Ethernet
* **Ouster Netwrok Setup and Config:**
  * Set the Jetson's Ethernet port to static IP `192.168.1.10`
    * check port with `ifconfig` (e.g. enP8pp1s0 connected to Ouster)
    * confirm connection and name with `nmcli connection show` name should be "Wired connection 1"
    * `ping os-<SERIAL NUMBER ON OUSTER>.local` This will show your ousters IP, I belive that all should be default to 192.168.1.11
    * `sudo nmcli connection modify "Wired connection 1" ipv4.addresses 192.168.1.10/24 ipv4.method manual` Changes ethernet port IP on jetson
    * `sudo nmcli connection down "Wired connection 1` Refresh
    * `sudo nmcli connection up "Wired connection 1`
    * `ping 192.168.1.11` Validate connection
  * in ~/NeuROAM/src/ouster-ros/ouster-ros/config/driver_params.yaml
    * set `sensor_hostname: '192.168.1.11'`
* **FLIR Config:**
  * in ~/NeuROAM/src/flir_camera_driver/spinnaker_camera_driver/launch
    * In driver_node.launch.py copy the blackfly_s parameters and paste them into multiple_cameras.launch.py camera_params
    * in multiple_cameras.launch.py Update `default_value` in `LaunchArg(cam_*_serial, default_value="'25094299'", description='....')` for both camera_0_serial and camera_1_serial to the corresponding serials on the cameras
* **Vectornav IMU Config:**
  * Nothing needs to be done as of now
* **Ublox GPS Config:**
  * in ~/NeuROAM/src/ublox/ublox_gps/config/zed_f9p.yaml
    * remove `svn_in` and everything below
    * set `tmode3 = 0`
    * sete `publish all: false` (these get rid of NACK)
  * in ~/NeuROAM/src/ublox/ublox_gps/launch/ublox_gps_node-launch.py
    * update `'c9_m8p_rover.yaml'` in `param_config = os.path.join(config_directory, 'c94_m8p_rover.yaml')` to `'zed_f9p.yaml`

## 4. Running Basic Example
* you should now be able to launch basic ros commands:
  * `cd ~/NeuROAM` 
  * `ros2 launch vectornav vectornav.launch.py`
  * `ros2 launch ublox_gps ublox_gps_node-launch.py`
  * `ros2 launch spinnaker_camera_driver multiple_cameras.launch.py`
  * `ros2 launch ouster_ros driver.launch.py`
* as well as basic topic echo and data visualization:
  * `ros2 topic echo /vectornav/imu`
  * `ros2 topic echo /ublox_gps_node/fix`
  * `rqt` in GUI go to plugins -> visualization -> image view (refresh and select camera image_raw)

