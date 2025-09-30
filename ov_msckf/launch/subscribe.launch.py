from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true", description="enable OpenVINS node"
    ),

    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="lunar_leaper",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="DEBUG",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="true",
        description="record the total state with calibration and features to a txt file",
    ),
    DeclareLaunchArgument(
    name="path_gt",
    default_value="/catkin_ws/runtime_groundtruth/construction1_full_gt.csv",
    description="Path to ground truth data file in ASL format"
    ),

    DeclareLaunchArgument(
        name="recorder_enable", default_value="true",
        description="enable ov_eval pose recorder"
    ),
    DeclareLaunchArgument(
        name="recorder_topic", default_value="/ov_msckf/poseimu",
        description="topic with estimated pose (e.g., /ov_msckf/poseimu or /ov_msckf/odom)"
    ),
    DeclareLaunchArgument(
        name="recorder_topic_type", default_value="PoseWithCovarianceStamped",
        description="PoseWithCovarianceStamped, PoseStamped, TransformStamped, and Odometry."
    ),
    DeclareLaunchArgument(
        name="recorder_output", default_value="/catkin_ws/estimated_trajectory/ov_runs/openvins_run_1.txt",
        description="output file for recorded trajectory"
    ),
        DeclareLaunchArgument(
        name="recorder_output", default_value="/catkin_ws/estimated_trajectory/ov_runs/openvins_run_1.txt",
        description="output file for recorded trajectory"
    ),
    # timing recorder (ROS 2)
    DeclareLaunchArgument(
        name="timing_recorder_enable", default_value="true",
        description="enable CPU/memory recorder"
    ),
    DeclareLaunchArgument(
        name="timing_nodes", default_value="/ov_msckf/run_subscribe_msckf",
        description="comma-separated node tokens, e.g. /ov_msckf/run_subscribe_msckf"
    ),
    DeclareLaunchArgument(
        name="timing_output", default_value="/catkin_ws/logging_files/psutil_log.txt",
        description="output file for timing/psutil logs"
    )
]

def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            config_path = os.path.join(
                            get_package_share_directory("ov_msckf"),
                            "config",config,"estimator_config.yaml"
                        )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path)
                    )
            ]
    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"config_path": config_path},
            {"path_gt": LaunchConfiguration("path_gt")}, 
        ],
    )
    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
            ],
    )


    recorder_output = LaunchConfiguration("recorder_output").perform(context)
    try:
        os.makedirs(os.path.dirname(recorder_output), exist_ok=True)
    except Exception as e:
        return [LogInfo(msg=f"ERROR creating recorder output dir: {e}")]


    recorder_node = Node(
        package="ov_eval",
        executable="pose_to_file_ros2",
        condition=IfCondition(LaunchConfiguration("recorder_enable")),
        output="screen",
        parameters=[
            {"topic": LaunchConfiguration("recorder_topic")},
            {"topic_type": LaunchConfiguration("recorder_topic_type")},
            {"output": LaunchConfiguration("recorder_output")},
        ],
    )

    recorder_timing_node = Node(
        package="ov_eval",
        executable="pid_ros2.py",
        condition=IfCondition(LaunchConfiguration("timing_recorder_enable")),
        output="screen",
        parameters=[
            {"nodes": LaunchConfiguration("timing_nodes")},
            {"output": LaunchConfiguration("timing_output")},
        ],
    )

    return [node1, node2, recorder_node, recorder_timing_node]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
