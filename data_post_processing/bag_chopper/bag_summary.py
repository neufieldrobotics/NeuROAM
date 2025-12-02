from pathlib import Path
# from rosbags.rosbag2 import Reader, Writer
import sys
import argparse
import json
import os
import yaml

# Import necessary libraries
from scipy.spatial.transform import Rotation as R

import sys

from mcap_ros2.decoder import DecoderFactory

from mcap_ros2.reader import make_reader
from mcap.writer import Writer
import tqdm
import matplotlib.pyplot as plt

def check_info(original_file, yaml_info):
    total_messages = 0
    topic_counts = {}

    with open(original_file, "rb") as f:
        print(f'reading {original_file}')

        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()

        for channel_id, channel in summary.channels.items():
            stats = summary.statistics.channel_message_counts.get(channel_id, 0)
            try:
                topic_counts[channel.topic] += stats
            except:
                topic_counts[channel.topic] = stats
            total_messages += stats
    f.close()

    # Check total message count
    if total_messages != yaml_info["total_messages"]:
        raise ValueError(
            f"Total message count mismatch: "
            f"{total_messages} != {yaml_info['total_messages']}"
        )
    # Check per-topic counts
    for topic, count in topic_counts.items():

        yaml_count = yaml_info["topic_wise_message"].get(topic)
        if yaml_count is None:
            if count != 0:
                raise ValueError(
                    f"Missing topic in yaml info but found in original file '{topic}'"
                )
        else:
            if yaml_count != count:
                raise ValueError(
                    f"Mismatch in topic '{topic}': {count} != {yaml_count}"
                )
    print("✅ Counts match with YAML file")

def save_to_yaml(info: dict, out_file: str):
    # Only keep total + topic counts in yaml
    data = {
        "total_messages": info["total_messages"],
        "topic_wise_message": info["topic_wise_message"]
    }
    with open(out_file, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)

def make_summary(bag_dir):
    total_messages = 0
    topic_counts = {}

    bag_files = os.listdir(bag_dir)
    for idx, bag_file in enumerate(bag_files):
        bag_path = os.path.join(bag_dir, bag_file)
        with open(bag_path, "rb") as f:
            print(f'reading {bag_path}')

            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            summary = reader.get_summary()

            for channel_id, channel in summary.channels.items():
                stats = summary.statistics.channel_message_counts.get(channel_id, 0)
                try:
                    topic_counts[channel.topic] += stats
                except:
                    topic_counts[channel.topic] = stats
                total_messages += stats
            f.close()

    print(topic_counts, total_messages)
    yaml_info = {
        'total_messages' : total_messages,
        'topic_wise_message' : topic_counts
    }

    yaml_file =  bag_dir[:-1] + '_info.yaml'
    print(f'Saving to file {yaml_file}')
    save_to_yaml(yaml_info, yaml_file)
    return yaml_info

def main():
    parser = argparse.ArgumentParser(description="Read bag files and generate summary and check with .")

    # Mandatory input
    parser.add_argument(
        "bag_dir",
        type=str,
        help="Path(s) to the input bag file(s)"
    )

    parser.add_argument(
        'original_file',
        type=str,
        help=''
    )

    args = parser.parse_args()

    # Access arguments
    bag_dir = args.bag_dir
    original_file = args.original_file
    
    print(f"Bag files: {bag_dir}")

    info = make_summary(bag_dir)
    check_info(original_file, info)
if __name__ == "__main__":
    main()
