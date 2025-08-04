#!/bin/bash

# List of valid payload hostnames
valid_payloads=("payload0" "payload1" "payload2" "payload3" "payload4")

# Get current hostname
hostname=$(hostname)

# Validate hostname
is_valid=false
for p in "${valid_payloads[@]}"; do
    if [[ "$hostname" == "$p" ]]; then
        is_valid=true
        break
    fi
done

if [[ "$is_valid" != "true" ]]; then
    echo "❌ ERROR: Hostname '$hostname' is not a recognized payload."
    echo "Valid payloads are: ${valid_payloads[*]}"
    exit 1
fi

# Get date and time
date=$(date +%Y%m%d)
time=$(date +%H%M)

# Output templated file names and directories
echo "=== TEMPLATE OUTPUTS FOR $hostname ==="
echo
echo "[Calibration directory]"
echo "~/data/calibration/${date}/"
echo
echo "[Calibration files]"
echo "lidar_imu_calibration_${hostname}_${date}_${time}.yaml"
echo "stereo_imu_calibration_${hostname}_${date}_${time}.yaml"
echo "stereo_camera_calibration_${hostname}_${date}_${time}.yaml"
echo "vectornav_imu_calibration_${hostname}_${date}_${time}.yaml"
echo
echo "[Local data copy dir]"
echo "/data/${hostname}/${date}/"
echo
echo "[Bag file]"
echo "${hostname}_${date}_${time}.bag"
echo
echo "[Notes file]"
echo "notes_${hostname}_${date}_${time}.txt"
echo
echo "[Storage server dir]"
echo "/data/${hostname}/${date}/"
