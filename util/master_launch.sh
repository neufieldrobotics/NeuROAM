#!/bin/bash

echo "Starting all sensor nodes in the background..."

source /opt/ros/humble/setup.bash
source ~/NeuROAM/install/setup.bash

echo "Launching VectorNav..."
ros2 launch vectornav vectornav.launch.py &
sleep 2

echo "Launching FLIR Cameras..."
ros2 launch spinnaker_synchronized_camera_driver follower_example.launch.py &
sleep 2

echo "Launching Ouster LiDAR..."
ros2 launch ouster_ros driver.launch.py &
sleep 2

echo "Launching GPS..."
ros2 launch ublox_gps ublox_gps_node-launch.py &

echo "All sensor nodes have been launched in the background."
echo "To stop all nodes, run 'killall ros2'"
