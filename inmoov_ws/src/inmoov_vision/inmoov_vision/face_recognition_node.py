#!/usr/bin/env python3
"""
face_recognition_node.py

ROS 2 node that performs real-time face recognition using Dlib.
It compares detected faces from a camera feed (subscribed topic) against a database of known individuals,
and publishes the recognized person's name (or "unknown"/"none") to the topic /recognized_person.

# Author: Alejandro Alonso Puig (https://github.com/aalonsopuig) + ChatGPT 4.1
Date: June 5, 2025
License: Apache 2.0
"""

# --- Standard libraries ---
import os        # For handling paths
import cv2       # OpenCV for image processing
import dlib      # Dlib for face detection and recognition
import pickle    # For loading serialized face encodings

# --- ROS 2 libraries ---
import rclpy                             # Core ROS 2 Python library
from rclpy.node import Node              # Base class for all ROS 2 nodes
from std_msgs.msg import String          # Message type for publishing recognized names
from sensor_msgs.msg import Image        # Message type for incoming image frames
from cv_bridge import CvBridge           # Utility to convert between ROS and OpenCV images


class FaceRecognitionNode(Node):
    """
    ROS 2 node that receives image frames via topic, detects faces,
    compares them with a known database, and publishes the recognition result.
    """

    def __init__(self):
        super().__init__('face_recognition_node')
        self.get_logger().info("Face Recognition Node started.")

        # --- Load precomputed facial embeddings ---
        base_dir = os.path.dirname(os.path.abspath(__file__))
        encodings_path = os.path.join(base_dir, 'data', 'encodings.pickle')
        with open(encodings_path, 'rb') as f:
            self.encodings = pickle.load(f)

        # --- Load required Dlib models ---
        models_dir = os.path.join(base_dir, 'models')

        self.detector = dlib.get_frontal_face_detector()
        self.shape_predictor = dlib.shape_predictor(
            os.path.join(models_dir, 'shape_predictor_68_face_landmarks.dat')
        )
        self.face_recognizer = dlib.face_recognition_model_v1(
            os.path.join(models_dir, 'dlib_face_recognition_resnet_model_v1.dat')
        )

        # --- ROS 2 interfaces ---
        self.publisher_ = self.create_publisher(String, '/recognized_person', 10)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        """
        Called automatically every time a new image arrives on /camera/image_raw.
        Performs face detection, recognition, and publishes results:
        - If a known face: publishes the name.
        - If an unknown face: publishes "unknown".
        - If no face detected: publishes "none".
        """
        try:
            # Convert ROS Image message to OpenCV BGR image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert to RGB (as required by Dlib)
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detect faces in the image
        faces = self.detector(rgb_image)

        # Prepare message to publish
        msg_out = String()

        if len(faces) == 0:
            # No faces detected: publish "none"
            self.get_logger().info("No face detected.")
            msg_out.data = "none"
            self.publisher_.publish(msg_out)
            return

        # Iterate over each detected face (only one expected per frame in this context)
        for face in faces:
            # Compute facial landmarks
            shape = self.shape_predictor(rgb_image, face)

            # Generate 128D descriptor for the face
            descriptor = self.face_recognizer.compute_face_descriptor(rgb_image, shape)
            descriptor_list = list(descriptor)

            # Match this descriptor against known encodings
            match_name, min_distance = self.compare_embeddings(descriptor_list)

            # Fill and publish the result message
            if match_name:
                self.get_logger().info(f"Recognized: {match_name} (distance={min_distance:.3f})")
                msg_out.data = match_name
            else:
                self.get_logger().info(f"Recognized: unknown (distance={min_distance:.3f})")
                msg_out.data = "unknown"

            self.publisher_.publish(msg_out)
            # In most use cases, only the first detected face is relevant, so break here
            break

    def compare_embeddings(self, query_embedding, threshold=0.6):
        """
        Compare a given face descriptor against all known descriptors.

        Args:
            query_embedding (list): 128D descriptor of the detected face
            threshold (float): Maximum Euclidean distance to consider a match

        Returns:
            (match_name, distance) if found, or (None, min_distance)
        """
        best_match = None
        min_dist = float('inf')

        for name, embeddings in self.encodings.items():
            for known_embedding in embeddings:
                dist = self.euclidean_distance(query_embedding, known_embedding)
                if dist < min_dist:
                    min_dist = dist
                    best_match = name

        if min_dist < threshold:
            return best_match, min_dist
        else:
            return None, min_dist

    @staticmethod
    def euclidean_distance(vec1, vec2):
        """
        Compute Euclidean distance between two 128D face descriptor vectors.
        """
        return sum((a - b) ** 2 for a, b in zip(vec1, vec2)) ** 0.5


def main(args=None):
    """
    Entry point: Initializes the node and starts spinning.
    """
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
