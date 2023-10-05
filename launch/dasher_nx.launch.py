import os 
from ament_index_python import get_package_share_directory, get_package_prefix
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
)
import launch_ros

def launch_setup(context: LaunchContext, args):
    """Remapping and launching the video_source node for each camera,
        where `args` is the number of cameras to be launched
        
        params:
        
        `context` (`LaunchContext`): Context of the launch description
        
        `args` (any): launch configuration
    """
    num_cams = context.perform_substitution(args)
    print("Number of launched cameras: "+num_cams)
    nodes=[]
    for idx in range(int(num_cams)):
        print(idx)
        nodes.append(launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            output='screen',
            namespace='cam'+str(idx),
            name="video_source"+str(idx),
            parameters=[
                {'resource': "csi://"+str(idx)},
                {'width': LaunchConfiguration('input_width')},
                {'height': LaunchConfiguration('input_height')},
                {'codec': 'h264'},
                {'loop': LaunchConfiguration('input_loop')},
                {'latency': LaunchConfiguration('input_latency')}
            ],
            remappings=[
                ('/video_output/raw', "/video_source/raw"+str(idx))
            ]
        ))
        nodes.append(
            launch_ros.actions.Node(package='image_transport', executable='republish', output='screen', name='republish', remappings=[
            ('in', '/cam'+str(idx)+'/raw'),
            ('out', '/cam'+str(idx)+'/image_raw')
        ], arguments=['raw'])
    
    )
    return nodes

def launch_setup_manual(context: LaunchContext,args):
    cam_idx = context.perform_substitution(args)
    nodes=[]    
    # print("Launching csi camera: "+args)
    # print("Launching csi camera args1: "+str(args1))
    if int(cam_idx) < 0:
        return 
    else:
        nodes.append(launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            output='screen',
            namespace='cam'+str(cam_idx),
            name="video_source"+str(cam_idx),
            parameters=[
                {'resource': "csi://"+str(cam_idx)},
                {'width': LaunchConfiguration('input_width')},
                {'height': LaunchConfiguration('input_height')},
                {'codec': 'h264'},
                {'loop': LaunchConfiguration('input_loop')},
                {'latency': LaunchConfiguration('input_latency')}
            ],
            remappings=[
                ('/video_output/raw', "/video_source/raw"+str(cam_idx))
            ])
        )
        return nodes

def generate_launch_description():

    # Declare launch arguments
    output_arg=DeclareLaunchArgument('output', default_value='display://0')
    output_codec_arg=DeclareLaunchArgument('output_codec', default_value='unknown')
    output_bitrate_arg=DeclareLaunchArgument('output_bitrate', default_value='0')
    topic_arg=DeclareLaunchArgument('topic', default_value='/video_source/raw')
    input_arg=DeclareLaunchArgument('input', default_value='csi://0')
    input_width_arg=DeclareLaunchArgument('input_width', default_value='0')
    input_height_arg=DeclareLaunchArgument('input_height', default_value='0')
    input_codec_arg=DeclareLaunchArgument('input_codec', default_value='h264')
    input_loop_arg=DeclareLaunchArgument('input_loop', default_value='0')
    input_latency_arg=DeclareLaunchArgument('input_latency', default_value='2000')
    num_cameras_args=DeclareLaunchArgument('num_cameras', default_value="2")
    
    cam0_args=DeclareLaunchArgument('cam0', default_value="-1")
    cam1_args=DeclareLaunchArgument('cam1', default_value="-1")
    cam2_args=DeclareLaunchArgument('cam2', default_value="-1")
    cam3_args=DeclareLaunchArgument('cam3', default_value="-1")
    cam4_args=DeclareLaunchArgument('cam4', default_value="-1")
    cam5_args=DeclareLaunchArgument('cam5', default_value="-1")
    
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch/foxglove_bridge_launch.xml",
            )
        ),
        launch_arguments={
            "num_threads": "6",
            "send_buffer_limit": "1000000",  # (1Mb)
            # "topic_whitelist": "['^/cam[0-9]/raw/compressed$','^/camera/.*','^/imu/.*','/light_control','/ultrasonics','/odom','/dasher_status','/cmd_vel','/gps','/utc','/tf']"
        }.items(),
    )

    ld=LaunchDescription([
        output_arg,
        output_codec_arg,
        output_bitrate_arg,
        topic_arg,
        input_arg,
        input_width_arg,
        input_height_arg,
        input_codec_arg,
        input_loop_arg,
        input_latency_arg,
        num_cameras_args,
        cam0_args,
        cam1_args,
        cam2_args,
        cam3_args,
        cam4_args,
        cam5_args,
        foxglove_bridge,
        # OpaqueFunction(function=launch_setup,args=[LaunchConfiguration('num_cameras')])
        OpaqueFunction(function=launch_setup_manual,args=[LaunchConfiguration('cam0')]),
        OpaqueFunction(function=launch_setup_manual,args=[LaunchConfiguration('cam1')]),
        OpaqueFunction(function=launch_setup_manual,args=[LaunchConfiguration('cam2')]),
        OpaqueFunction(function=launch_setup_manual,args=[LaunchConfiguration('cam3')]),
        OpaqueFunction(function=launch_setup_manual,args=[LaunchConfiguration('cam4')]),
        OpaqueFunction(function=launch_setup_manual,args=[LaunchConfiguration('cam5')])
    ])
    return ld
