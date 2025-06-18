#!/usr/bin/env python3
"""
face_tracker_node.py

ROS 2 node that detects faces using OpenCV and Haar cascades,
publishes their relative position to /face_position and the raw camera frame to /camera/image_raw.

Author: Alejandro Alonso Puig (https://github.com/aalonsopuig) + ChatGPT 4.1
License: Apache 2.0
Date: June 5, 2025
"""

import rclpy  # ROS 2 client library for Python
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from geometry_msgs.msg import Point  # Message type for publishing face position
from sensor_msgs.msg import Image  # Message type for publishing image frames
from cv_bridge import CvBridge  # Bridge to convert between ROS and OpenCV image formats
from ament_index_python.packages import get_package_share_directory  # Locate package directories
import cv2  # OpenCV library for computer vision
import os  # For path manipulations

class FaceTrackerNode(Node):
    """
    ROS 2 node that detects the largest human face in the image using Haar cascades,
    calculates its position relative to the image center, and publishes it to /face_position.
    Also publishes the current video frame as /camera/image_raw.
    """

    def __init__(self):
        super().__init__('face_tracker_node')

        # Declare parameters with default values
        self.declare_parameter('video_device', 0)  # Default camera device index is 0
        self.declare_parameter('processing_frequency', 10.0)  # Default processing rate in Hz
        self.declare_parameter('show_debug_window', True)  # Whether to show OpenCV debug window

        # Read parameters from the ROS 2 parameter server
        device_index = self.get_parameter('video_device').get_parameter_value().integer_value
        frequency = self.get_parameter('processing_frequency').get_parameter_value().double_value
        self.show_debug = self.get_parameter('show_debug_window').get_parameter_value().bool_value

        # Publisher for 3D point indicating face position in the frame
        self.publisher_ = self.create_publisher(Point, 'face_position', 10)

        # Publisher for raw camera images using sensor_msgs/Image
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 1)  # Reduce queue size to 1 to minimize lag
        self.bridge = CvBridge()  # Bridge between OpenCV and ROS image formats

        # Load Haar cascade classifier from the package's haarcascades folder
        package_share_directory = get_package_share_directory('inmoov_vision')
        haar_path = os.path.join(package_share_directory, 'haarcascades', 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(haar_path)

        # Attempt to open the specified video capture device
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Limit buffer to avoid latency

        if not self.cap.isOpened():
            self.get_logger().error(f'Camera could not be opened at index {device_index}')
            return

        # Set up periodic timer to process frames at the specified frequency
        self.timer = self.create_timer(1.0 / frequency, self.process_frame)

    def process_frame(self):
        """
        Called periodically. Captures the most recent frame (clearing camera buffer),
        detects the largest face, publishes its relative position as a Point message,
        and publishes the current image as an Image message.
        """
        # Clear possible stale frames by reading a few times
        for _ in range(5):
            self.cap.read()
        ret, frame = self.cap.read()  # Final fresh frame
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        # Convert frame to grayscale for more efficient face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces in the grayscale image
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        height, width = gray.shape  # Get image dimensions
        msg = Point()  # Initialize message to publish

        if len(faces) > 0:
            # Select the largest face detected (based on area)
            x, y, w, h = max(faces, key=lambda rect: rect[2] * rect[3])
            cx = x + w // 2  # X coordinate of face center
            cy = y + h // 2  # Y coordinate of face center

            # Normalize x and y coordinates to [-1, 1] range
            msg.x = (cx - width // 2) / (width // 2)
            msg.y = (cy - height // 2) / (height // 2)
            msg.z = float(w * h)  # Face area as z

            # Annotate image if debug display is enabled
            if self.show_debug:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(frame, f"Face at ({msg.x:.2f}, {msg.y:.2f})", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Face size (z): {msg.z:.0f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            # No face detected — publish neutral zeroed values
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0

        # Publish the face position and image
        self.publisher_.publish(msg)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        # Optionally display the processed frame
        if self.show_debug:
            cv2.imshow('Face Tracker', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Shutdown requested by user')
                rclpy.shutdown()

    def destroy_node(self):
        """
        Cleanup resources on shutdown: release camera and close OpenCV windows.
        """
        self.cap.release()
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """
    Entry point: initializes the ROS 2 node and starts spinning.
    """
    rclpy.init(args=args)
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
