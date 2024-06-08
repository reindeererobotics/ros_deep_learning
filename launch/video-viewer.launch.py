from calendar import c
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
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
)
from datetime import datetime
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessStart,
)

def init_csi_stream(idx, width, height):
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return ExecuteProcess(
        cmd=[
            [
                "video-viewer ",
                f"csi://{idx} "
                f"--input-width={width} "
                f"--input-height={height}  "
                "--headless "
                "--input-flip=rotate-180 "
                "--input-codec=h265 "
                "--output-codec=h265 "
                f"rtsp://@1234/csi_{idx} ",
                # "/jetson-inference/ros/recordings/${current_time}_csi0.mp4",
            ]
        ],
        output="screen",
        log_cmd=True,
        shell=True,
    )

def generate_launch_description():
    
    viewer0 = init_csi_stream(0, 1920, 1080)
    viewer1 = init_csi_stream(1, 1920, 1080)
    viewer2 = init_csi_stream(2, 1920, 1080)
    viewer3 = init_csi_stream(3, 1920, 1080)
    viewer4 = init_csi_stream(4, 1920, 1080)
    viewer5 = init_csi_stream(5, 1920, 1080)

    return launch.LaunchDescription(
        [
            viewer0,
            viewer1,
            viewer2,
            viewer3,
            viewer4,
            viewer5,
        ]
    )
