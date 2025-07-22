#!/bin/bash


# Set the baud rate for the serial device
sudo stty -F /dev/ttyTHS1 115200

sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
source ~/NeuROAM/install/setup.bash

# Trap SIGINT (Ctrl+C) and SIGTERM to kill all background processes
cleanup() {
    echo "Caught Ctrl+C, stopping all ros2 launch processes..."
    kill 0  # Kills all child processes in the current process group
    wait
    echo "All processes terminated."
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "Launching zenoh daemon..."
ros2 run rmw_zenoh_cpp rmw_zenohd &
zenohd_pid=$!

# Start 4 ros2 launch nodes in the background
echo "Starting ROS 2 launch files..."

ros2 launch vectornav vectornav.launch.py &
pid1=$!
ros2 launch ouster_ros driver.launch.py &
pid2=$!
ros2 launch ublox_gps ublox_gps_node-launch.py &
pid3=$!
ros2 launch spinnaker_synchronized_camera_driver follower_example.launch.py  &
pid4=$!

# Wait for all background processes
wait $pid1 $pid2 $pid3 $pid4
