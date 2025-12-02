from pathlib import Path
# from rosbags.rosbag2 import Reader, Writer
import sys
import argparse
import json
import os

# Import necessary libraries
from scipy.spatial.transform import Rotation as R

import sys
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from mcap.reader import make_reader
from mcap.writer import Writer
import tqdm
import matplotlib.pyplot as plt

output_dir = '/media/goku/10C79C6626EE33ED/NeuRAOM20251119/payload4/'

def get_timestamp_idx(timestamp, partition):
    for k,v in partition.items():
        if timestamp >= v[0] and timestamp < v[1]:
            return k
    print(f'No timestamp for {timestamp}')
    # sys.exit(1)

def chop(bag_files, timestamps, topics, topic_partitions):

    bag_partitions = []
    bag_partition_idx = []
    bag_partition_dict = {}
    idx = 0
    for timestamp_partitions in timestamps: # Check every timestamp partition for bag
        for k in timestamp_partitions.keys(): # Check every timestamp partition
            for t in topic_partitions:
                partition_name = f'{k}_{t}'
                print(partition_name, idx)
                bag_partitions.append(partition_name)
                bag_partition_idx.append(idx)
                bag_partition_dict[partition_name] = idx
                idx+=1

    print(f'Found {len(bag_files)} bags')
    partition_writers = {}  # keep track of open writers

    for idx, bag_file in enumerate(bag_files):
        # try:

        print(f'Reading bag {bag_file}' )
        with open(bag_file, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            total = reader.get_summary().statistics.message_count 

            for schema, channel, message, decoded in tqdm.tqdm(reader.iter_decoded_messages(), total=total):

                # Find partition based on timestamp
                time_suffix = get_timestamp_idx(message.log_time, timestamps[idx])                    
                
                # Handle for topics not specified
                try:
                    topics[channel.topic]
                except:
                    continue

                # A message can be written to multiple bags. Iterate and write to each bag
                for topic_suffix in topics[channel.topic]:
                    partition_name = f'{time_suffix}_{topic_suffix}'
                    partition_path = f'{output_dir}/{partition_name}.mcap'

                    if partition_name not in partition_writers:
                        f_out = open(partition_path, "wb")
                        writer = Writer(f_out)
                        writer.start()
                        partition_writers[partition_name] = (
                            writer, f_out, {}, {}  # schema_map, channel_map
                        )
                        bag_partition_dict[partition_name] = partition_path

                    writer, _, schema_map, channel_map = partition_writers[partition_name]

                    # Register schema only once
                    schema_key = (schema.name, schema.encoding, schema.data) if schema else None
                    if schema_key not in schema_map:
                        schema_id = writer.register_schema(
                            name=schema.name,
                            encoding=schema.encoding,
                            data=schema.data,
                        ) if schema else None
                        schema_map[schema_key] = schema_id
                    schema_id = schema_map[schema_key]

                    # Register channel only once
                    channel_key = channel.topic
                    if channel_key not in channel_map:
                        channel_id = writer.register_channel(
                            topic=channel.topic,
                            message_encoding=channel.message_encoding,
                            schema_id=schema_id,
                        )
                        channel_map[channel_key] = channel_id
                    channel_id = channel_map[channel_key]

                    # Write message
                    writer.add_message(
                        channel_id=channel_id,
                        log_time=message.log_time,
                        data=message.data,
                        publish_time=message.publish_time,
                    )

        print(f'Completed reading bag {idx})')
    for writer, f_out, _, _ in partition_writers.values():
        writer.finish()
        f_out.close()
                        
# 1. Check if all bag files in the provided txt file exist.
def check_bags(bag_paths: str):
    print(f"Checking for bag files in {bag_paths}")
    bagfiles = []
    try:
        with open(bag_paths, 'r') as file:
            bagfiles = file.readlines()
            bagfiles = [x[:-1] for x in bagfiles]
    except FileNotFoundError:
        print(f"Error: The file '{bag_paths}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

    bagfiles = [bagfile for bagfile in bagfiles if not bagfile.strip().startswith('#')]
    for bagfile in bagfiles:
        if not os.path.isfile(bagfile):
            print(f"Error: Bag file '{bagfile}' does not exist.")
            sys.exit(1)
        else:
            print(f"Bag file '{bagfile}' found.")
    return bagfiles

# 2. Check if the timestamps JSON file structure is valid and the timestamps are correctly associated with the bag suffixes.
def check_timestamps(timestamps_file: str, bagfiles: list):
    print("Checking timestamps...")

    try:
        with open(timestamps_file, 'r') as f:
            timestamps_data = json.load(f)
    except Exception as e:
        print(f"Error loading timestamps file: {e}")
        return

    # List to store the flattened timestamps with their corresponding bag suffix
    flattened_timestamps = []

    # Process the nested JSON and flatten it
    for idx, (bag, suffixes) in enumerate(timestamps_data.items()):
        prev_timestamp = 0
        timestamp_dict = {}
        for bag_suffix, timestamp in suffixes.items():
            # Construct the key as per the format: "bag_<index>suffix<suffix_number>"
            key = f"{bag}_{bag_suffix}"

            timestamp_dict[key] = [prev_timestamp, timestamp]
            prev_timestamp = timestamp

        # Store the result in the desired format
        timestamp_dict[key][1] = 1796406181000000000
        flattened_timestamps.append(timestamp_dict)
    # print(flattened_timestamps)

    # Return the list of formatted strings
    if len(bagfiles) != (idx+1) :
        print(f'Unequal number of bags{len(bagfiles)} and timestamps{len(flattened_timestamps)}')
        sys.exit(1)
    return flattened_timestamps


# 3. Check if the topics JSON file structure is valid and create a hash table with topics as the keys.
def check_topics(topics_file: str):

    # Move every topic to a partition
    print("Checking topics...")
    
    try:
        with open(topics_file, 'r') as f:
            topics_data = json.load(f)
    except Exception as e:
        print(f"Error loading topics file: {e}")
        return

    topic_hash_table = {}
    partitions = [] # keys refer to bag partitions / suffixes
    for key, topic_list in topics_data.items():
        if not isinstance(topic_list, list):
            print(f"Error: Topics for key '{key}' should be a list.")
            continue

        for topic in topic_list:
            try:
                topic_hash_table[topic].append(key)
            except:
                topic_hash_table[topic] = [key]

        partitions.append(key)
    # Print out the hash table
    for k,v in topic_hash_table.items():
        print(f'Topic {k} will be added in partitions {v}')
    return topic_hash_table, partitions

def main():
    parser = argparse.ArgumentParser(description="Process bag files with optional timestamps and topics.")

    # Mandatory input
    parser.add_argument(
        "bagfiles",
        type=str,
        help="Path(s) to the input bag file(s)"
    )

    # Optional: timestamps
    parser.add_argument(
        "--timestamps",
        type=str,
        help="Path to a file containing timestamps or a comma-separated list of timestamps"
    )

    # Optional: topics
    parser.add_argument(
        "--topics",
        type=str,
        help="Comma-separated list of topics to filter (e.g. /topic1,/topic2)"
    )

    args = parser.parse_args()

    # Access arguments
    bagfiles = args.bagfiles
    timestamps = args.timestamps
    topics = args.topics

    print(f"Bag files: {bagfiles}")
    print(f"Timestamps: {timestamps}")
    print(f"Topics: {topics}")

    # TODO: Your processing logic here
    bagfiles = check_bags(bagfiles)
    timestamp_dict = check_timestamps(timestamps, bagfiles)
    topic_dict, topic_partitions = check_topics(topics)

    chop(bagfiles, timestamp_dict, topic_dict, topic_partitions)

if __name__ == "__main__":
    main()
