#!/usr/bin/env zsh

# require one argument: the bag file or directory containing the .mcap
if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <bag_file_or_directory>"
    exit 1
fi

# set warning that this script is set up for alan's computer (username: alan)
if [[ "$USER" != "alan" ]]; then
    echo "⚠️ WARNING: This script is set up for user 'alan'."
    echo "If you are not 'alan', please adjust the paths accordingly."
fi

conda activate neuroam
export LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"

# process the bag, visualize the data, and save to /tmp/imu_data.pkl
/home/alan/anaconda3/envs/neuroam/bin/python /home/alan/NeuROAM/util/imu/plot_imu_data.py "$1"

/home/alan/anaconda3/envs/neuroam/bin/python /home/alan/NeuROAM/util/imu/imu_eskf_gravity_zupt.py /tmp/imu_data.pkl --dt-max 0.05 --plot