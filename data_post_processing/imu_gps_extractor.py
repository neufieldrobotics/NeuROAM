#!/usr/bin/env python3
import argparse
import os
import csv
from collections import defaultdict
import pandas as pd
import yaml

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import tqdm

def load_vectornav_config(yaml_file):
    """Load VectorNav topics configuration from YAML file."""
    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)
    
    enabled_topics = {}
    for topic_name, topic_info in config['topics'].items():
        if topic_info.get('enabled', False):
            enabled_topics[topic_name] = topic_info
    
    return enabled_topics

def get_message_count_for_topics(reader, target_topics):
    summary = reader.get_summary()
    if summary is None or summary.statistics is None:
        return 0
    
    channel_counts = summary.statistics.channel_message_counts
    channels = summary.channels
    
    count = 0
    for channel_id, message_count in channel_counts.items():
        if channels[channel_id].topic in target_topics:
            count += message_count
    return count

def extract_field_value(decoded, field_path):
    """Extract field value from decoded message using dot notation."""
    parts = field_path.split('_')
    obj = decoded
    
    # Handle special cases
    if field_path.startswith('orientation_'):
        return getattr(decoded.orientation, parts[1])
    elif field_path.startswith('angular_velocity_'):
        return getattr(decoded.angular_velocity, parts[2])
    elif field_path.startswith('linear_acceleration_'):
        return getattr(decoded.linear_acceleration, parts[2])
    elif field_path.startswith('magnetic_field_'):
        return getattr(decoded.magnetic_field, parts[2])
    elif field_path.startswith('time_ref_'):
        return getattr(decoded.time_ref, parts[2])
    elif field_path.startswith('ypr_'):
        return getattr(decoded.yawpitchroll, parts[1])
    elif field_path.startswith('quaternion_'):
        return getattr(decoded.quaternion, parts[1])
    elif field_path.startswith('angularrate_'):
        return getattr(decoded.angularrate, parts[1])
    elif field_path.startswith('position_') and hasattr(decoded, 'pose'):
        return getattr(decoded.pose.pose.position, parts[1])
    elif field_path.startswith('position_covariance_'):
        idx = int(parts[2])
        return decoded.position_covariance[idx]
    elif field_path.startswith('linear_') and hasattr(decoded, 'twist'):
        return getattr(decoded.twist.twist.linear, parts[1])
    elif field_path.startswith('angular_') and hasattr(decoded, 'twist'):
        return getattr(decoded.twist.twist.angular, parts[1])
    elif field_path.startswith('pos_lla_'):
        return getattr(decoded.poslla, parts[2])
    elif field_path.startswith('pos_ecef_'):
        return getattr(decoded.posecef, parts[2])
    elif field_path.startswith('vel_ned_'):
        return getattr(decoded.velned, parts[2])
    elif field_path.startswith('vel_ecef_'):
        return getattr(decoded.velecef, parts[2])
    elif field_path.startswith('vel_body_'):
        return getattr(decoded.velbody, parts[2])
    elif field_path.startswith('pos_u_'):
        return getattr(decoded.posu, parts[2])
    elif field_path.startswith('mag_ecef_'):
        return getattr(decoded.magecef, parts[2])
    elif field_path.startswith('accel_ecef_'):
        return getattr(decoded.accelecef, parts[2])
    elif field_path.startswith('linear_accel_body_'):
        return getattr(decoded.linearaccelbody, parts[3])
    else:
        # Direct field access
        return getattr(decoded, field_path)

def extract_vectornav_data(reader, topics_config):
    """Extract VectorNav data based on config."""
    
    topic_names = list(topics_config.keys())
    
    # Initialize data storage
    imu_data = defaultdict(lambda: {'timestamp_sec': None, 'timestamp_nsec': None})
    
    # Add fields for each enabled topic
    for topic_name, topic_info in topics_config.items():
        for field in topic_info['fields']:
            key = f"{topic_name.replace('/', '_')[1:]}_{field}"
            imu_data[list(imu_data.keys())[0] if imu_data else 0][key] = None
    
    total = get_message_count_for_topics(reader, topic_names)
    print(f"Processing {total} VectorNav messages from {len(topic_names)} topics...")
    
    for schema, channel, message, decoded in tqdm.tqdm(
        reader.iter_decoded_messages(topics=topic_names), 
        total=total,
        desc="Reading VectorNav data"
    ):
        timestamp = decoded.header.stamp.sec * 1_000_000_000 + decoded.header.stamp.nanosec
        
        imu_data[timestamp]['timestamp_sec'] = decoded.header.stamp.sec
        imu_data[timestamp]['timestamp_nsec'] = decoded.header.stamp.nanosec
        
        # Extract fields for this topic
        topic_info = topics_config[channel.topic]
        for field in topic_info['fields']:
            key = f"{channel.topic.replace('/', '_')[1:]}_{field}"
            try:
                imu_data[timestamp][key] = extract_field_value(decoded, field)
            except Exception as e:
                print(f"Warning: Could not extract {field} from {channel.topic}: {e}")
                imu_data[timestamp][key] = None
    
    return imu_data

def build_multiindex_columns(topics_config):
    """Build MultiIndex columns from topics config."""
    columns_list = [('timestamp', 'sec'), ('timestamp', 'nsec')]
    
    for topic_name, topic_info in topics_config.items():
        for field in topic_info['fields']:
            columns_list.append((topic_name, field))
    
    return pd.MultiIndex.from_tuples(columns_list)

def write_imu_csv_hierarchical(imu_data, topics_config, output_csv):
    print(f"\nWriting {len(imu_data)} VectorNav rows to {output_csv}...")
    
    columns = build_multiindex_columns(topics_config)
    
    rows = []
    for timestamp in tqdm.tqdm(sorted(imu_data.keys()), desc="Building DataFrame"):
        d = imu_data[timestamp]
        row = [d['timestamp_sec'], d['timestamp_nsec']]
        
        for topic_name, topic_info in topics_config.items():
            for field in topic_info['fields']:
                key = f"{topic_name.replace('/', '_')[1:]}_{field}"
                row.append(d.get(key))
        
        rows.append(row)
    
    df = pd.DataFrame(rows, columns=columns)
    df.to_csv(output_csv, index=False)
    
    print(f"Successfully wrote hierarchical VectorNav CSV with {df.columns.nlevels} header rows")

def extract_gps_data(reader, topic):
    gps_data = []
    
    total = get_message_count_for_topics(reader, [topic])
    print(f"\nProcessing {total} GPS messages...")
    
    for schema, channel, message, decoded in tqdm.tqdm(
        reader.iter_decoded_messages(topics=[topic]), 
        total=total,
        desc="Reading GPS data"
    ):
        gps_data.append({
            'timestamp_sec': decoded.header.stamp.sec,
            'timestamp_nsec': decoded.header.stamp.nanosec,
            'status': decoded.status.status,
            'latitude': decoded.latitude,
            'longitude': decoded.longitude,
            'altitude': decoded.altitude
        })
    
    return gps_data

def write_gps_csv(gps_data, output_csv):
    print(f"\nWriting {len(gps_data)} GPS rows to {output_csv}...")
    
    fieldnames = ['timestamp_sec', 'timestamp_nsec', 'status', 'latitude', 'longitude', 'altitude']
    
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for row in tqdm.tqdm(gps_data, desc="Writing GPS CSV"):
            writer.writerow(row)
    
    print(f"Successfully wrote GPS data to {output_csv}")

def extract_all_data(bag_file, config_file, imu_output_csv, gps_output_csv):
    # Load VectorNav configuration
    print(f"Loading VectorNav configuration from {config_file}...")
    topics_config = load_vectornav_config(config_file)
    
    if not topics_config:
        print("ERROR: No topics enabled in configuration file!")
        return
    
    print(f"Enabled topics: {list(topics_config.keys())}")
    
    # Extract VectorNav data
    with open(bag_file, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        imu_data = extract_vectornav_data(reader, topics_config)
    
    # Extract GPS data
    gps_topic = '/ublox_gps_node/fix'
    with open(bag_file, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        gps_data = extract_gps_data(reader, gps_topic)
    
    # Write CSV files
    write_imu_csv_hierarchical(imu_data, topics_config, imu_output_csv)
    write_gps_csv(gps_data, gps_output_csv)
    
    print(f"\n{'='*80}")
    print(f"Extraction complete!")
    print(f"VectorNav CSV: {imu_output_csv} ({len(imu_data)} rows, hierarchical format)")
    print(f"GPS CSV: {gps_output_csv} ({len(gps_data)} rows)")
    print(f"{'='*80}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract VectorNav and GPS data from ROS2 bag to CSV files.")
    parser.add_argument("bag_file", help="Path to the .mcap ROS2 bag file")
    parser.add_argument("--config", help="Path to VectorNav topics YAML config", default="vectornav_topics.yaml")
    parser.add_argument("--imu-output", help="VectorNav CSV output path", default=None)
    parser.add_argument("--gps-output", help="GPS CSV output path", default=None)
    
    args = parser.parse_args()
    file_path = os.path.abspath(args.bag_file)
    base_name = os.path.splitext(os.path.basename(file_path))[0]
    base_dir = os.path.dirname(file_path)
    
    imu_output_csv = args.imu_output if args.imu_output else os.path.join(base_dir, f"{base_name}_imu.csv")
    gps_output_csv = args.gps_output if args.gps_output else os.path.join(base_dir, f"{base_name}_gps.csv")
    
    extract_all_data(file_path, args.config, imu_output_csv, gps_output_csv)