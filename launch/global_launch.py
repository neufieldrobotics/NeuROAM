from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
    SetEnvironmentVariable,#Added for cyclonedds
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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

    # Declare launch argument for bag name
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='my_bag',
        description='Name for the rosbag file (without extension)'
    )

    record_rosbag = LaunchConfiguration('record_rosbag')
    bag_name = LaunchConfiguration('bag_name')

    
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
        '/parameter_events', '/rosout', '/ouster/os_driver/transition_event',
        '/vectornav/raw/common', '/vectornav/raw/time', '/vectornav/time_startup',
        '/vectornav/time_gps', '/vectornav/raw/imu', '/vectornav/time_syncin',
        '/vectornav/raw/gps', '/vectornav/time_pps', '/vectornav/imu',
        '/vectornav/gnss', '/vectornav/raw/attitude', '/vectornav/imu_uncompensated',
        '/vectornav/raw/ins', '/vectornav/magnetic', '/vectornav/raw/gps2',
        '/vectornav/velocity_aiding', '/vectornav/temperature', '/vectornav/pressure',
        '/vectornav/velocity_body', '/vectornav/pose', '/tf_static', '/diagnostics',
        '/navstatus', '/navcov', '/navclock', '/aidalm', '/aideph', '/nmea', '/rtcm',
        '/ublox_gps_node/navpvt', '/ublox_gps_node/fix', '/ublox_gps_node/fix_velocity',
        '/navstate', '/monhw', '/rxmrtcm', '/timtm2', '/interrupt_time', '/rxmsfrb',
        '/rxmraw',
        '/cam_sync/cam0/meta', '/cam_sync/cam0/image_raw', '/cam_sync/cam0/camera_info',
        '/cam_sync/cam1/meta', '/cam_sync/cam1/image_raw', '/cam_sync/cam1/camera_info',
        '/ouster/metadata', '/ouster/imu', '/ouster/points', '/ouster/telemetry'
    ]

    rosbag_record = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record', '-s', 'mcap',
                    '-o', PathJoinSubstitution(['/home/roam1/data/', bag_name]),
                    '--max-cache-size', '6442450944', '--storage-preset-profile', 'fastwrite',
                ] + rosbag_topics,
                output='screen'
            )
        ],
        condition=IfCondition(record_rosbag)
    )

    return LaunchDescription([
        record_rosbag_arg,
        bag_name_arg,
        rmw_zenohd_process,
        delayed_launch,
        rosbag_record
    ])
