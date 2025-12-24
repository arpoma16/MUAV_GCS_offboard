#!/usr/bin/env python3
"""
Launch file for video_stream_compress node.
This file demonstrates how to launch the video stream compression node with different configurations.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for video stream compress node."""

    # Declare launch arguments
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='640',
        description='Width of the output frame'
    )

    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='480',
        description='Height of the output frame'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )

    source_arg = DeclareLaunchArgument(
        'source',
        default_value='0',
        description='Video source: 0 for camera, 1 for video file'
    )

    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID (for source=0)'
    )

    path_video_arg = DeclareLaunchArgument(
        'path_video',
        default_value='',
        description='Path to video file (for source=1)'
    )

    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG compression quality (0-100)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )

    # Create node
    video_stream_compress_node = Node(
        package='muav_gcs_offboard',
        executable='video_stream_compress',
        name='video_stream_compress',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'fps': LaunchConfiguration('fps'),
            'source': LaunchConfiguration('source'),
            'camera_id': LaunchConfiguration('camera_id'),
            'path_video': LaunchConfiguration('path_video'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
        }]
    )

    return LaunchDescription([
        frame_width_arg,
        frame_height_arg,
        fps_arg,
        source_arg,
        camera_id_arg,
        path_video_arg,
        jpeg_quality_arg,
        namespace_arg,
        video_stream_compress_node
    ])
