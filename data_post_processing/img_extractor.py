#!/usr/bin/env python3
import argparse
import os
import cv2
import numpy as np

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed
# from joblib import Parallel, delayed

NUM_THREADS = 32

# @profile
def write_image(decoded, filename):
    np_arr = np.frombuffer(decoded.data, np.uint8)
    bayer_img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BAYER_RG2RGB)
    
    cv2.imwrite(filename, color_img)
    return None

def get_message_count_for_topic(reader, target_topic):
        summary = reader.get_summary()

        if summary is None or summary.statistics is None:
            print("MCAP file does not contain summary statistics.")
            return 0

        # The statistics record contains channel_message_counts, which maps channel IDs to counts.
        channel_counts = summary.statistics.channel_message_counts

        # We also need the channels dictionary to map channel IDs to topic names.
        channels = summary.channels

        count = 0
        for channel_id, message_count in channel_counts.items():
            if channels[channel_id].topic == target_topic:
                count += message_count
        
        return count

def save_images_from_bag(bag_file):
    
    topic0_name = '/cam_sync/cam0/image_raw/compressed'
    topic1_name = '/cam_sync/cam1/image_raw/compressed'
    # skip_count = 0

    topic_dirs = {
        topic0_name: cam0_dir,
        topic1_name: cam1_dir
    }

    cam_count = {
        topic0_name: 0,
        topic1_name: 0
    }

    with open(bag_file, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        total = get_message_count_for_topic(reader, topic0_name)
        total += get_message_count_for_topic(reader, topic1_name)
        # mini_batch = []
        
        # """
        with ThreadPoolExecutor(max_workers=NUM_THREADS) as executor:
            futures = []
    
            for schema, channel, message, decoded in tqdm.tqdm(reader.iter_decoded_messages(topics=[topic0_name, topic1_name]), total=total):
                if (channel.topic == topic1_name) or (channel.topic == topic0_name):
                    timestamp = decoded.header.stamp.sec * 1_000_000_000 + decoded.header.stamp.nanosec
                    img_dir = topic_dirs[channel.topic]
                    filename = os.path.join(img_dir, f"{timestamp}.png")

                    # mini_batch.append((decoded, filename))

                    futures.append(executor.submit(write_image, decoded, filename))
                    cam_count[channel.topic] += 1

                    # Process completed futures in batches to limit memory usage
                    if len(futures) >= 1_000:
                        for future in tqdm.tqdm(as_completed(futures)):
                            try:
                                future.result()  # Process the result
                            except Exception as e:
                                print(f"Error saving image: {e}")
                        futures.clear()  # Clear the list of futures


            # Optional: wait for all to complete and catch exceptions
            for future in tqdm.tqdm(as_completed(futures), total=len(futures), desc="Saving images"):
                try:
                    future.result()
                except Exception as e:
                    print(f"Error saving image: {e}")

        
            # # Process all image tasks in parallel
            # results = Parallel(n_jobs=NUM_THREADS, backend="loky", batch_size=16)(
            #     delayed(write_image)(data, fname) for data, fname in tqdm.tqdm(tasks, desc="Saving images")
            # )
        # """
    print(f"Saved {cam_count[topic0_name]} images to {cam0_dir}")
    print(f"Saved {cam_count[topic1_name]} images to {cam1_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract and debayer images from ROS2 bag (MCAP).")
    parser.add_argument("bag_file", help="Path to the .mcap ROS2 bag file")
    
    # Add an optional argument for the output directory
    parser.add_argument(
        "-o", 
        "--output-dir", 
        help="Optional output directory path. If not specified, a directory named '<bag_file_base>_pcd' will be created in the bag file's location.",
        default=None # We set default to None and handle the logic later
    )
    
    args = parser.parse_args()
    file_path = os.path.abspath(args.bag_file)
    base_dir = os.path.dirname(file_path)
    
    # Determine the final output directory
    if args.output_dir:
        # Use the provided output directory path
        cam0_dir = os.path.abspath(args.output_dir) + 'cam_0'
        cam1_dir = os.path.abspath(args.output_dir) + 'cam_1'
    else:
        # Define new directory names
        cam0_dir = f"{base_dir}_cam0"
        cam1_dir = f"{base_dir}_cam1"

    os.makedirs(cam0_dir, exist_ok=True)
    os.makedirs(cam1_dir, exist_ok=True)

    print(f'Saving to {cam0_dir}, {cam1_dir}')
    save_images_from_bag(file_path)

