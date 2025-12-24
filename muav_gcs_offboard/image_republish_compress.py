#!/usr/bin/env python3
"""
@Description: This node subscribes to an uncompressed image topic and republishes it as compressed JPEG.
@Input Topic: Configurable (e.g., /px4_3/camera/image_raw)
@Output Topic: <input_topic>/compressed (e.g., /px4_3/camera/image_raw/compressed)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class ImageRepublishCompress(Node):
    """Node for subscribing to raw images and republishing as compressed JPEG."""

    def __init__(self):
        super().__init__('image_republish_compress')

        # Declare parameters
        self.declare_parameter('input_topic', '/px4_3/camera/image_raw')
        self.declare_parameter('output_topic', '')  # If empty, uses input_topic/compressed

        # Get parameter values
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        # Set output topic
        if not self.output_topic:
            self.output_topic = f'{self.input_topic}/compressed'

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create subscriber
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )

        # Create publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            self.output_topic,
            10
        )

        # Statistics
        self.frame_count = 0
        self.error_count = 0

        # Log configuration
        self.get_logger().info('=' * 60)
        self.get_logger().info('Image Republish Compress Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Input topic:  {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info('Format: JPEG (using cv_bridge)')
        self.get_logger().info('=' * 60)

    def image_callback(self, msg):
        """Callback function for image subscription."""
        try:
            # Use CvBridge to convert and compress in one step
            # This uses the cv_bridge built into ROS2 which handles OpenCV internally
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'),
                dst_format='jpg'
            )

            # Keep original header (timestamp, frame_id)
            compressed_msg.header = msg.header

            # Publish compressed image
            self.publisher.publish(compressed_msg)

            # Update statistics
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(
                    f'Processed {self.frame_count} frames '
                    f'(Errors: {self.error_count})'
                )

        except CvBridgeError as e:
            self.error_count += 1
            self.get_logger().error(f'CvBridge error: {str(e)}')
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)

    try:
        image_republish_node = ImageRepublishCompress()
        rclpy.spin(image_republish_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'image_republish_node' in locals():
            image_republish_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
