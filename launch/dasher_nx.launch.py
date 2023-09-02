from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
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
        OpaqueFunction(function=launch_setup,args=[LaunchConfiguration('num_cameras')])
    ])
    return ld