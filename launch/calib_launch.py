from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
    SetEnvironmentVariable,  # Added for cyclonedds
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch argument for optional rosbag recording
    record_rosbag_arg = DeclareLaunchArgument(
        'record_rosbag',
        default_value='false',
        description='Set to true to record rosbag'
    )

    record_rosbag = LaunchConfiguration('record_rosbag')

    # Start rmw_zenohd daemon
    rmw_zenohd_process = ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        output='screen'
    )

    # Delayed component launch (5s delay)
    delayed_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('vectornav'),
                        'launch',
                        'vectornav.launch.py'
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('ublox_gps'),
                        'launch',
                        'ublox_gps_node-launch.py'
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('spinnaker_synchronized_camera_driver'),
                        'launch',
                        'follower_example.launch.py'
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('ouster_ros'),
                        'launch',
                        'driver.launch.py'
                    )
                )
            ),
        ]
    )

    # List of topics to record (cleaned, no leading space)
    rosbag_topics = [
    '/vectornav/imu',
    '/cam_sync/cam0/image_raw', 
    '/cam_sync/cam1/image_raw',
    '/ouster/points',
    ]

    rosbag_record = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record', '-s', 'mcap',
                    '-o', '/home/roam1/data/calib_bag', '--max-cache-size', '2147483648', '--storage-preset-profile', 'fastwrite',
                ] + rosbag_topics,
                output='screen'
            )
        ],
        condition=IfCondition(record_rosbag)
    )

    def get_domain_id():
        computer_hostname = os.uname()[1]
        hostname_to_domain_id = {
            "payload0": 0,
            "payload1": 1,
            "payload2": 2,
            "payload3": 3,
            "payload4": 4,
        }
        domain_id = hostname_to_domain_id.get(computer_hostname, -1)
        if domain_id == -1:
            raise RuntimeError(
                f"Unknown hostname {computer_hostname}, " "do not know ROS_DOMAIN_ID."
            )
        return domain_id

    # set ZENOH parameters
    zenoh_env = SetEnvironmentVariable(
        name="ZENOH_CONFIG_OVERRIDE",
        value="transport/link/tx/queue/congestion_control/drop/wait_before_drop=1000000",
    )

    # Set ROS_DOMAIN_ID based on the computer hostname
    ros_domain_id = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID", value=str(get_domain_id())
    )


    return LaunchDescription([
        zenoh_env,
        ros_domain_id,
        record_rosbag_arg,
        rmw_zenohd_process,
        delayed_launch,
        rosbag_record
    ])

