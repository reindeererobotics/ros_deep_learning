import launch
import os
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    LocalSubstitution,
    TextSubstitution,
)
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
)
from datetime import datetime
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessStart,
)


def init_csi_stream(context: launch.LaunchContext, *args):
    camera_args = LaunchConfiguration("cameras").perform(
        context
    )  # reference: https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L24
    camera_width = LaunchConfiguration("camera_width").perform(context)
    camera_height = LaunchConfiguration("camera_height").perform(context)

    cam_indicies = [
        int(idx) for idx in camera_args.split(",")
    ]  # Unpack the list of camera indices into an array, casting from str to int. i.e. "0,1,2,3,4,5" --> [0,1,2,3,4,5]

    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    dasher_vid_viewer = [
        ExecuteProcess(
            cmd=[
                [
                    "video-viewer ",
                    f"csi://{idx} ",
                    f"--input-width={camera_width} ",
                    f"--input-height={camera_height}  ",
                    "--headless ",
                    "--input-flip=rotate-180 ",
                    "--input-codec=h265 ",
                    "--output-codec=h265 ",
                    f"rtsp://@:123{idx}/csi_{idx} ",
                    # "/jetson-inference/ros/recordings/${current_time}_csi0.mp4",
                ]
            ],
            output="screen",
            log_cmd=True,
            shell=True,
        )
        for idx in cam_indicies
    ]
    return dasher_vid_viewer


def generate_launch_description():

    # Declare an argument "cameras" that takes a list of camera indices
    camera_list_arg = DeclareLaunchArgument(
        "cameras",
        default_value="0,1,2,3,4,5",
        description="List of camera indices to initialize",
    )

    camera_width_arg = DeclareLaunchArgument(
        "camera_width",
        default_value="1920",
        description="camera width",
    )

    camera_height_arg = DeclareLaunchArgument(
        "camera_height",
        default_value="1080",
        description="camera height",
    )

    return launch.LaunchDescription(
        [
            camera_list_arg,
            camera_width_arg,
            camera_height_arg,
            OpaqueFunction(
                function=init_csi_stream,
            ),
        ]
    )
