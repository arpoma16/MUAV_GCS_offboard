#!/usr/bin/env python3
"""
Launch file for image_republish_compress node.
Subscribes to raw image topics and republishes as compressed JPEG.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for image republish compress node."""

    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/px4_3/camera/image_raw',
        description='Input topic with uncompressed images'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/px4_3/camera/image_compressed',
        description='Output topic for compressed images (empty = input_topic/compressed)'
    )


    # Create node
    image_republish_compress_node = Node(
        package='muav_gcs_offboard',
        executable='image_republish_compress',
        name='image_republish_compress',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
        }]
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        image_republish_compress_node
    ])
