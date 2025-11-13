from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
    SetEnvironmentVariable,  # Added for cyclonedds
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


COMPUTER_HOSTNAME = os.uname()[1]
HOSTNAME_TO_DOMAIN_ID = {
    "payload0": 0,
    "payload1": 1,
    "payload2": 2,
    "payload3": 3,
    "payload4": 4,
}

if COMPUTER_HOSTNAME not in HOSTNAME_TO_DOMAIN_ID:
    raise RuntimeError(
        f"Unknown hostname {COMPUTER_HOSTNAME}, should be one of: {list(HOSTNAME_TO_DOMAIN_ID.keys())}"
    )

import datetime

now = datetime.datetime.now()
BAG_FNAME = f"{COMPUTER_HOSTNAME}_{now.strftime('%Y%m%d_%H%M')}"

DOMAIN_ID = HOSTNAME_TO_DOMAIN_ID[COMPUTER_HOSTNAME]

RECORD_SEPARATE = False
RECORD_COMPRESSED_IMAGES = True


def generate_launch_description():

    # set ZENOH parameters
    zenoh_env = SetEnvironmentVariable(
        name="ZENOH_CONFIG_OVERRIDE",
        value="transport/link/tx/queue/congestion_control/drop/wait_before_drop=1000000",
    )

    # Set ROS_DOMAIN_ID based on the computer hostname
    ros_domain_id = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value=str(DOMAIN_ID))

    # Declare launch argument for optional rosbag recording
    record_rosbag_arg = DeclareLaunchArgument(
        "record_rosbag",
        default_value="false",
        description="Set to true to record rosbag",
    )

    # Declare launch argument for bag name
    bag_name_arg = DeclareLaunchArgument(
        "bag_name",
        default_value=BAG_FNAME,
        description="Name for the rosbag file (without extension)",
    )

    record_rosbag = LaunchConfiguration("record_rosbag")
    bag_name = LaunchConfiguration("bag_name")

    # Start rmw_zenohd daemon
    rmw_zenohd_process = ExecuteProcess(
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"], output="screen"
    )

    # Delayed component launch (5s delay)
    delayed_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("vectornav"),
                        "launch",
                        "vectornav.launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ublox_gps"),
                        "launch",
                        "ublox_gps_node-launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(
                            "spinnaker_synchronized_camera_driver"
                        ),
                        "launch",
                        "follower_example.launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ouster_ros"),
                        "launch",
                        "driver.launch.py",
                    )
                ),
                launch_arguments={
                    'params_file': os.path.join(
                        get_package_share_directory("ouster_ros"),
                        "config",
                        f"driver_params_{COMPUTER_HOSTNAME}.yaml"
                    )
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("system_health_monitor"),
                        "launch",
                        "health_monitor.launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("doodle_monitor"),
                        "launch",
                        "doodle_monitor.launch.py",
                    )
                )
            ),
        ],
    )

    # List of topics to record (cleaned, no leading space)
    small_data_topics = [
        "/parameter_events",
        "/rosout",
        "/vectornav/raw/common",
        "/vectornav/raw/time",
        "/vectornav/time_startup",
        "/vectornav/time_gps",
        "/vectornav/raw/imu",
        "/vectornav/time_syncin",
        "/vectornav/raw/gps",
        "/vectornav/time_pps",
        "/vectornav/imu",
        "/vectornav/gnss",
        "/vectornav/raw/attitude",
        "/vectornav/imu_uncompensated",
        "/vectornav/raw/ins",
        "/vectornav/magnetic",
        "/vectornav/raw/gps2",
        "/vectornav/velocity_aiding",
        "/vectornav/temperature",
        "/vectornav/pressure",
        "/vectornav/velocity_body",
        "/vectornav/pose",
        "/tf_static",
        # diagnostics
        "/diagnostics",
        "/computer/diagnostics",
        "/node/diagnostics",
        # doodle labs info
        "/doodle_monitor/iperf_result",
        "/doodle_monitor/raw",
        "/doodle_monitor/sys/cpu_load",
        "/doodle_monitor/sys/freemem",
        "/doodle_monitor/sys/localtime",
        "/doodle_monitor/noise",
        "/doodle_monitor/activity",
        "/doodle_monitor/lna_status",
        "/doodle_monitor/sta_status",
        "/doodle_monitor/mesh_status",
        "/doodle_monitor/peer_list",
        # other info
        "/navstatus",
        "/navcov",
        "/navclock",
        "/aidalm",
        "/aideph",
        "/nmea",
        "/rtcm",
        "/ublox_gps_node/navpvt",
        "/ublox_gps_node/fix",
        "/ublox_gps_node/fix_velocity",
        "/navstate",
        "/monhw",
        "/rxmrtcm",
        "/timtm2",
        "/interrupt_time",
        "/rxmsfrb",
        "/rxmraw",
    ]

    image_topic_name = (
        "image_raw/compressed" if RECORD_COMPRESSED_IMAGES else "image_raw"
    )

    def get_cam_topics(cam_id):
        return [
            f"/cam_sync/cam{cam_id}/meta",
            f"/cam_sync/cam{cam_id}/{image_topic_name}",
            f"/cam_sync/cam{cam_id}/camera_info",
        ]

    cam0_topics = get_cam_topics(0)
    cam1_topics = get_cam_topics(1)

    ouster_topics = [
        # "/ouster/os_driver/transition_event",
        "/ouster/metadata",
        "/ouster/imu",
        "/ouster/points",
        # "/ouster/telemetry",
        # "/ouster/scan",
        "/ouster/reflec_image",
        "/ouster/signal_image",
        "/ouster/nearir_image",
        # "/ouster/range_image",
    ]

    def make_record_action(topics, record_bag_name, suffix=""):
        path_join = (
            ["/home/neuroam/data/", record_bag_name, suffix]
            if suffix
            else ["/home/neuroam/data/", record_bag_name]
        )
        return TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "bag",
                        "record",
                        "-s",
                        "mcap",
                        "-o",
                        PathJoinSubstitution(path_join),
                        "--max-bag-size",
                        "3000000000",  # GB
                        "--max-bag-duration",
                        "3600",  # seconds
                        "--max-cache-size",
                        # "1073741824",
                        "6442450944",
                        "--storage-preset-profile",
                        "fastwrite",
                        "--qos-profile-overrides-path",
                        "/home/neuroam/NeuROAM/launch/custom_qos.yaml",
                    ]
                    + topics,
                    output="screen",
                )
            ],
            condition=IfCondition(record_rosbag),
        )

    if RECORD_SEPARATE:
        record_actions = [
            make_record_action(small_data_topics, bag_name, "small"),
            make_record_action(cam0_topics, bag_name, "cam0"),
            make_record_action(cam1_topics, bag_name, "cam1"),
            make_record_action(ouster_topics, bag_name, "ouster"),
        ]
    else:
        record_actions = [
            make_record_action(
                small_data_topics + cam0_topics + cam1_topics + ouster_topics,
                bag_name,
            )
        ]

    return LaunchDescription(
        [
            zenoh_env,
            ros_domain_id,
            record_rosbag_arg,
            bag_name_arg,
            rmw_zenohd_process,
            delayed_launch,
            *record_actions,
        ]
    )
