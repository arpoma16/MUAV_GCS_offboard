#!/usr/bin/env python3
"""
@Description: This node publishes compressed images (JPEG format) from a USB camera or video file.
              Adapted from ROS1 C++ implementation to ROS2 Python.
@Topic: Publishes to /<namespace>/video_stream_compress
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class VideoStreamCompress(Node):
    """Node for capturing and publishing compressed video frames."""

    def __init__(self):
        super().__init__('video_stream_compress')

        # Declare parameters with default values
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('fps', 30)
        self.declare_parameter('path_video', '')
        self.declare_parameter('source', 0)  # 0 for camera, 1 for video file
        self.declare_parameter('camera_id', 0)  # Camera device ID
        self.declare_parameter('jpeg_quality', 80)  # JPEG compression quality (0-100)

        # Get parameter values
        self.frame_height = self.get_parameter('frame_height').value
        self.frame_width = self.get_parameter('frame_width').value
        self.fps = self.get_parameter('fps').value
        self.path_video = self.get_parameter('path_video').value
        self.source = self.get_parameter('source').value
        self.camera_id = self.get_parameter('camera_id').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # Get namespace
        self.namespace = self.get_namespace()

        # Create topic path
        if self.namespace == '/':
            topic_path = '/video_stream_compress'
        else:
            topic_path = f'{self.namespace}/video_stream_compress'

        # Create publisher
        self.publisher = self.create_publisher(CompressedImage, topic_path, 10)

        # Log configuration
        self.get_logger().info(f'Video Stream Compress Node Started')
        self.get_logger().info(f'FPS: {self.fps}')
        self.get_logger().info(f'Source: {self.source} (0=camera, 1=video)')
        self.get_logger().info(f'Resolution: {self.frame_width} x {self.frame_height}')
        self.get_logger().info(f'JPEG Quality: {self.jpeg_quality}%')
        self.get_logger().info(f'Publishing to: {topic_path}')

        # Initialize video capture
        self.cap = None
        if self.source == 0:
            # Use camera
            self.get_logger().info(f'Opening camera {self.camera_id}')
            self.cap = cv2.VideoCapture(self.camera_id)
        else:
            # Use video file
            self.get_logger().info(f'Opening video file: {self.path_video}')
            self.cap = cv2.VideoCapture(self.path_video)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video source!')
            return

        # Set JPEG compression parameters
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        # Create timer for publishing frames
        timer_period = 1.0 / self.fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Timer created with period: {timer_period:.3f}s')

    def timer_callback(self):
        """Callback function to capture and publish frames."""
        # Read frame from video capture
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().error('Failed to capture frame')
            # If video file reached end, loop back to start
            if self.source == 1:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # Resize frame to specified dimensions
        resized_frame = cv2.resize(
            frame,
            (self.frame_width, self.frame_height),
            interpolation=cv2.INTER_LINEAR
        )

        # Compress frame to JPEG format
        ret, buffer = cv2.imencode('.jpg', resized_frame, self.jpeg_params)

        if not ret:
            self.get_logger().error('Failed to encode frame to JPEG')
            return

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()

        # Publish message
        self.publisher.publish(msg)

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)

    try:
        video_stream_node = VideoStreamCompress()
        rclpy.spin(video_stream_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'video_stream_node' in locals():
            video_stream_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
