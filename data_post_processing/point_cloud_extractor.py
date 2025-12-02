#!/usr/bin/env python3
import argparse
import os
import cv2
import numpy as np

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
# from joblib import Parallel, delayed
import numpy as np
# import sensor_msgs_py.point_cloud2 as pc2

NUM_THREADS = 32

_DATATYPE_MAP = {
    PointField.INT8:    np.int8,
    PointField.UINT8:   np.uint8,
    PointField.INT16:   np.int16,
    PointField.UINT16:  np.uint16,
    PointField.INT32:   np.int32,
    PointField.UINT32:  np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}

xyz_dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])

def mcap_dynamic_to_pointcloud(dynamic_msg):
    data = np.frombuffer(dynamic_msg.data, dtype=np.uint8)
    points = data.reshape(-1, dynamic_msg.point_step).view(xyz_dtype)
    return points

def pc2_to_numpy(pc2_msg):
    """
    Convert PointCloud2 to a structured numpy array *without* ros2_numpy or sensor_msgs_py.
    Uses only raw byte buffer.
    """
    # Build dtype from fields
    dtype_list = []
    for f in pc2_msg.fields:
        base = _DATATYPE_MAP[f.datatype]
        if f.count == 1:
            dtype_list.append((f.name, base))
        else:
            dtype_list.append((f.name, base, (f.count,)))

    dtype = np.dtype(dtype_list)
    xyz_dtype = np.dtype(dtype_list[:3])

    # Interpret raw data buffer
    data = np.frombuffer(pc2_msg.data, dtype=np.uint8)

    # Each point occupies pc2_msg.point_step bytes → reshape then reinterpret
    points = data.reshape(-1, pc2_msg.point_step).view(xyz_dtype)

    return points


def mcap_dynamic_to_pointfield(dynamic_msg):
    field = PointField()
    field.name = dynamic_msg.name
    field.offset = dynamic_msg.offset
    field.datatype = dynamic_msg.datatype
    field.count = dynamic_msg.count
    return field

def mcap_dynamic_to_pointcloud2(dynamic_msg):
    pc2 = PointCloud2()
    
    # Copy all ROS fields (header + metadata)
    pc2.header = Header()
    pc2.header.stamp = Time()
    pc2.header.stamp.sec = dynamic_msg.header.stamp.sec
    pc2.header.stamp.nanosec = dynamic_msg.header.stamp.nanosec
    pc2.header.frame_id = dynamic_msg.header.frame_id

    pc2.height = dynamic_msg.height
    pc2.width = dynamic_msg.width
    pc2.fields = []
    for f in dynamic_msg.fields:
        pc2.fields.append(mcap_dynamic_to_pointfield(f))
    pc2.is_bigendian = dynamic_msg.is_bigendian
    pc2.point_step = dynamic_msg.point_step
    pc2.row_step = dynamic_msg.row_step
    pc2.data = dynamic_msg.data
    pc2.is_dense = dynamic_msg.is_dense

    return pc2

# @profile
def write_point_cloud(point_cloud, filename):
    np.savez(filename, xyz=point_cloud)
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
    
    topic_name = '/ouster/points'

    point_count = {
        topic_name: 0,
    }

    with open(bag_file, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        total = get_message_count_for_topic(reader, topic_name)
        # """
        with ThreadPoolExecutor(max_workers=NUM_THREADS) as executor:
            futures = []
    
            for schema, channel, message, decoded in tqdm.tqdm(reader.iter_decoded_messages(topics=[topic_name]), total=total):
                # if channel.topic == topic_name:
                    timestamp = decoded.header.stamp.sec * 1_000_000_000 + decoded.header.stamp.nanosec
                    filename = os.path.join(pcd_dir, f"{timestamp}")

                    # ros_msg = mcap_dynamic_to_pointcloud2(decoded)
                    # point_cloud = pc2_to_numpy(ros_msg)
                    
                    point_cloud = mcap_dynamic_to_pointcloud(decoded)
                    
                    futures.append(executor.submit(write_point_cloud, point_cloud, filename))

                    point_count[channel.topic] += 1

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
        # """
    print(f"Saved {point_count[topic_name]} images to {pcd_dir}")

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
        pcd_dir = os.path.abspath(args.output_dir)
    else:
        # Define the default directory name if no argument was provided
        pcd_dir = f"{base_dir}_pcd"
    
    os.makedirs(pcd_dir, exist_ok=True)

    print(f'Saving to {pcd_dir}')
    save_images_from_bag(file_path)