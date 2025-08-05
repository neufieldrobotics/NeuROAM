#!/usr/bin/env bash
# check_ros.sh
# Checks that a list of ROS2 nodes are up and certain topics have publishers.

set -eo pipefail

# --- CONFIG: fill in your exact node names here ---
expected_nodes=(
  "/vectornav"
  "/ublox_gps_node"
  "/cam_sync/cam0"
  "/cam_sync/cam1"
  "/os_driver"
)

# --- CONFIG: topics you expect to have at least one publisher ---
expected_topics=(
  "/parameter_events"
  "/rosout"
  "/ouster/os_driver/transition_event"
  "/vectornav/raw/common"
  "/vectornav/raw/time"
  "/vectornav/time_startup"
  "/vectornav/time_gps"
  "/vectornav/raw/imu"
  "/vectornav/time_syncin"
  "/vectornav/raw/gps"
  "/vectornav/time_pps"
  "/vectornav/imu"
  "/vectornav/gnss"
  "/vectornav/raw/attitude"
  "/vectornav/imu_uncompensated"
  "/vectornav/raw/ins"
  "/vectornav/magnetic"
  "/vectornav/raw/gps2"
  "/vectornav/velocity_aiding"
  "/vectornav/temperature"
  "/vectornav/pressure"
  "/vectornav/velocity_body"
  "/vectornav/pose"
  "/tf_static"
  "/diagnostics"
  "/navstatus"
  "/navcov"
  "/navclock"
  "/aidalm"
  "/aideph"
  "/nmea"
  "/rtcm"
  "/ublox_gps_node/navpvt"
  "/ublox_gps_node/fix"
  "/ublox_gps_node/fix_velocity"
  "/navstate"
  "/monhw"
  "/rxmrtcm"
  "/timtm2"
  "/interrupt_time"
  "/rxmsfrb"
  "/rxmraw"
  "/cam_sync/cam0/meta"
  "/cam_sync/cam0/image_raw"
  "/cam_sync/cam0/camera_info"
  "/cam_sync/cam1/meta"
  "/cam_sync/cam1/image_raw"
  "/cam_sync/cam1/camera_info"
  "/ouster/metadata"
  "/ouster/imu"
  "/ouster/points"
  "/ouster/telemetry"
)

missing_nodes=()
missing_topics=()

echo "------ Checking nodes ------"
running_nodes=$(ros2 node list)
for node in "${expected_nodes[@]}"; do
  if echo "$running_nodes" | grep -qw "$node"; then
    printf "  ✓ Node \"%s\" found\n" "$node"
  else
    printf "  ✗ Node \"%s\" NOT found\n" "$node"
    missing_nodes+=("$node")
  fi
done

echo
echo "------ Checking topics ------"
for topic in "${expected_topics[@]}"; do
  # Option A: just check if the topic exists in the list
  if ros2 topic list | grep -qx "$topic"; then
    # Optionally check number of publishers:
    pub_count=$(ros2 topic info "$topic" 2>/dev/null \
      | grep Publishers: | awk '{print $2}')
    if [[ "$pub_count" -ge 1 ]]; then
      printf "  ✓ Topic \"%s\" has %d publisher(s)\n" "$topic" "$pub_count"
    else
      printf "  ✗ Topic \"%s\" exists but has no publishers\n" "$topic"
      missing_topics+=("$topic")
    fi
  else
    printf "  ✗ Topic \"%s\" NOT found\n" "$topic"
    missing_topics+=("$topic")
  fi
done

echo
# summary
if [ "${#missing_nodes[@]}" -eq 0 ] && [ "${#missing_topics[@]}" -eq 0 ]; then
  echo "All checks passed"
  exit 0
else
  echo "---- Summary of missing ----"
  [ "${#missing_nodes[@]}" -ne 0 ] && {
    echo "  Missing nodes: ${missing_nodes[*]}"
  }
  [ "${#missing_topics[@]}" -ne 0 ] && {
    echo "  Missing topics: ${missing_topics[*]}"
  }
  exit 1
fi
