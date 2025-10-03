import os
from pathlib import Path

from mcap.reader import make_reader
import sys
import time
target_freqs = {
#   "/cam_sync/cam0/image_raw" : 20,
#   "/cam_sync/cam1/image_raw" : 20,
  "/cam_sync/cam0/image_raw/compressed" : 20,
  "/cam_sync/cam1/image_raw/compressed" : 20,
  "/ouster/points" : 10,
  "/vectornav/imu" : 200,
  "/ublox_gps_node/fix: " : 1,
}

def get_mcap_stats(mcap_file):
    with open(mcap_file, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()

        if summary is None:
            print("No summary section in this MCAP file. Try iterating messages instead.")
            return

        # Extract start and end times in seconds
        start_time = summary.statistics.message_start_time / 1e9
        end_time = summary.statistics.message_end_time / 1e9
        duration = end_time - start_time

        # Per-channel details
        print("Topics:")
        for channel_id, channel in summary.channels.items():
            msgs = summary.statistics.channel_message_counts.get(channel_id, 0)
            freq = msgs/duration

            if channel.topic in target_freqs.keys():

                target = target_freqs[channel.topic]
                lower = target * 0.99
                upper = target * 1.01

                if lower <= freq <= upper:
                    print(f"{channel.topic} {freq:.2f} ✅")
                else:
                    print(f"{channel.topic} {freq:.2f} ❌")

def get_last_subdir(directory):
    """Find the most recently modified subdirectory that does NOT contain metadata.yaml"""
    subdirs = [
        d for d in Path(directory).iterdir() 
        if d.is_dir() and not (d / "metadata.yaml").exists()
    ]
    if not subdirs:
        print(f'No active recording bagfile found. All dirs have complteted recording')
        return None

    last_subdir = max(subdirs, key=lambda d: d.stat().st_mtime)
    return last_subdir

def get_files_sorted_by_time(directory):
    """Get all files in directory sorted by modification time (newest first)"""
    files = [f for f in Path(directory).iterdir() if f.is_file()]
    files_sorted = sorted(files, key=lambda f: f.stat().st_mtime, reverse=True)
    return files_sorted

def get_last_mcap(files):
    # Get second most recent file
    if len(files) >= 2:
        second_most_recent = files[1]
        # print(f"Second most recent file: {second_most_recent}")
        return second_most_recent
    elif len(files) == 1:
        print("Only 1 file found, no second most recent file")
        return None
    else:
        print("No files found in subdirectory")
        return None

def wait(wait_time):
    wait_time
    print(f"Waiting for {wait_time}s...")

    for remaining in range(wait_time, 0, -1):
        sys.stdout.write(f"\r{remaining:2d}s remaining")
        sys.stdout.flush()
        time.sleep(1)

# POINT TO DATA DIRECTORY
def main(directory="/home/neuroam/data"):

    # WAIT TO START RECORDING
    wait(30)

    # LATEST BAG RECORDING WHICH HAS NO metadata.yaml
    # Find last made subdirectory
    last_subdir = get_last_subdir(directory)
    
    if last_subdir is None:
        print(f"No active recording subdirectories found in {directory}")
        return

    # DATA-COLLECITON DIR payloadX_YYYYMMDD......    
    print(f"Most recent subdirectory: {last_subdir}")
    processed_files = set()

    while True:
        # List all files in it
        files = get_files_sorted_by_time(last_subdir)

        # Get second last file
        latest_file = get_last_mcap(files)

        if latest_file is not  None:

            # If we haven't seen this file yet, process it
            if latest_file not in processed_files:
                print(f'Reading file {latest_file}')
                try:
                    get_mcap_stats(latest_file)
                    processed_files.add(latest_file)
                # if we read metadata.yaml while ending the recording
                except:
                    if 'metadata.yaml' in str(latest_file):
                        break
                    else:
                        print(f'Error getting mcap stats for bag{latest_file}')

        if len(files) > 1:
            if 'metadata.yaml' in str(files[1]):
                break

        wait(15)
    for f in processed_files:
        print(f'Processed file: {f}')

# Usage
if __name__ == "__main__":
    directory = "/home/neuroam/data"  # Current directory, change as needed
    second_file = main(directory)