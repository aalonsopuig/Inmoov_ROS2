#!/usr/bin/env python3
# face_tracking_behavior_node.py
#
# Author: Alejandro Alonso Puig (https://github.com/aalonsopuig) + ChatGPT 4.1
# License: Apache 2.0
#
# ROS 2 node for face tracking behavior on the InMoov humanoid robot.
# Subscribes to:
#   /face_position (geometry_msgs/Point)
#   /recognized_person (std_msgs/String)
# Publishes:
#   pan (/rothead) and tilt (/neck) commands (std_msgs/Int16) to the head servos
#   eye horizontal (/eye_y) and eye vertical (/eye_x) commands (std_msgs/Int16)
# Adjusts head servo angles incrementally and eye servo angles directly based on
# face position. Resets to center if no face detected beyond a threshold or if
# recognized_person is "none".

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Point  # Message type for face position
from std_msgs.msg import Int16  # Message type for servo commands
from std_msgs.msg import String  # Message type for recognized person

class FaceTrackingBehaviorNode(Node):
    """
    ROS 2 node that performs real-time face tracking behavior for the InMoov robot.
    Subscribes to face and recognition topics, manages head pan/tilt and eye movements.
    """

    # Tilt (neck) servo limits (changed NECK_MIN to 50)
    NECK_MIN = 50
    NECK_MAX = 170
    NECK_CENTER = 110

    # Pan (rothead) servo limits
    ROTHEAD_MIN = 50
    ROTHEAD_MAX = 130
    ROTHEAD_CENTER = 90

    # Eye horizontal (eye_y) servo limits
    EYE_Y_MIN = 60
    EYE_Y_MAX = 100
    EYE_Y_CENTER = 80

    # Eye vertical (eye_x) servo limits
    EYE_X_MIN = 70
    EYE_X_MAX = 90
    EYE_X_CENTER = 80

    # Number of consecutive zero messages before resetting to center
    LOST_FACE_THRESHOLD = 10

    def __init__(self):
        """
        Initializes the node:
        - Subscribes to /face_position and /recognized_person
        - Sets up publishers for head and eye servos
        - Initializes current positions and state variables
        """
        super().__init__('face_tracking_behavior_node')

        # Subscriptions
        self.create_subscription(Point, '/face_position', self.face_position_callback, 10)
        self.create_subscription(String, '/recognized_person', self.person_callback, 10)

        # Publishers for head servos
        self.neck_pub = self.create_publisher(Int16, '/neck', 10)
        self.rothead_pub = self.create_publisher(Int16, '/rothead', 10)
        # Publishers for eye servos
        self.eye_y_pub = self.create_publisher(Int16, '/eye_y', 10)
        self.eye_x_pub = self.create_publisher(Int16, '/eye_x', 10)

        # Gains for head adjustments (incremental)
        self.pan_gain = 10
        self.tilt_gain = 20
        # Gains for eye mapping (direct)
        self.eye_y_gain = self.EYE_Y_MAX - self.EYE_Y_CENTER  # 20
        self.eye_x_gain = self.EYE_X_CENTER - self.EYE_X_MIN  # 10

        # Initialize current positions
        self.current_rothead = self.ROTHEAD_CENTER
        self.current_neck = self.NECK_CENTER
        self.current_eye_y = self.EYE_Y_CENTER
        self.current_eye_x = self.EYE_X_CENTER

        # State variables
        self.no_face_count = 0
        self.recognized_person = ''

        # Inform that node has started
        self.get_logger().info('Node started and ready.')

    def person_callback(self, msg: String):
        """Callback for /recognized_person topic, updates recognized_person state."""
        self.recognized_person = msg.data
        # Debug recognized person value
        self.get_logger().debug(f"Recognized person: {self.recognized_person}")

    def face_position_callback(self, msg: Point):
        """
        Callback for face position messages.
        - If recognized_person is "none", treat as no face.
        - Tracks consecutive zero messages to reset head/eyes.
        - Updates head angles incrementally and eye angles directly.
        """
        # Determine offsets, treating explicit "none" as no face
        if self.recognized_person.lower() == 'none':
            x, y = 0.0, 0.0
        else:
            x, y = msg.x, msg.y

        # No face detected logic
        if x == 0 and y == 0:
            self.no_face_count += 1
            # Debug the no-face counter
            self.get_logger().debug(f"No face count: {self.no_face_count}")
            if self.no_face_count >= self.LOST_FACE_THRESHOLD:
                # Reset head to center
                if self.current_rothead != self.ROTHEAD_CENTER:
                    self.publish_servo(self.rothead_pub, self.ROTHEAD_CENTER)
                    self.current_rothead = self.ROTHEAD_CENTER
                if self.current_neck != self.NECK_CENTER:
                    self.publish_servo(self.neck_pub, self.NECK_CENTER)
                    self.current_neck = self.NECK_CENTER
                # Reset eyes to center
                if self.current_eye_y != self.EYE_Y_CENTER:
                    self.publish_servo(self.eye_y_pub, self.EYE_Y_CENTER)
                    self.current_eye_y = self.EYE_Y_CENTER
                if self.current_eye_x != self.EYE_X_CENTER:
                    self.publish_servo(self.eye_x_pub, self.EYE_X_CENTER)
                    self.current_eye_x = self.EYE_X_CENTER
                # Inform that reset occurred
                self.get_logger().info('No face detected: head and eyes reset to center.')
            return
        else:
            # Face present, reset the no-face counter
            self.no_face_count = 0

        # Head adjustments (incremental)
        delta_pan = int(-x * self.pan_gain)
        delta_tilt = int(-y * self.tilt_gain)
        new_rothead = self.clamp(self.current_rothead + delta_pan, self.ROTHEAD_MIN, self.ROTHEAD_MAX)
        new_neck = self.clamp(self.current_neck + delta_tilt, self.NECK_MIN, self.NECK_MAX)
        if new_rothead != self.current_rothead:
            self.publish_servo(self.rothead_pub, new_rothead)
            self.current_rothead = new_rothead
        if new_neck != self.current_neck:
            self.publish_servo(self.neck_pub, new_neck)
            self.current_neck = new_neck

        # Eye movements (direct mapping)
        desired_eye_y = int(self.clamp(self.EYE_Y_CENTER + x * self.eye_y_gain,
                                       self.EYE_Y_MIN, self.EYE_Y_MAX))
        desired_eye_x = int(self.clamp(self.EYE_X_CENTER - y * self.eye_x_gain,
                                       self.EYE_X_MIN, self.EYE_X_MAX))
        if desired_eye_y != self.current_eye_y:
            self.publish_servo(self.eye_y_pub, desired_eye_y)
            self.current_eye_y = desired_eye_y
        if desired_eye_x != self.current_eye_x:
            self.publish_servo(self.eye_x_pub, desired_eye_x)
            self.current_eye_x = desired_eye_x

        # Debug the computed positions
        self.get_logger().debug(
            f"Face pos x={x:.2f}, y={y:.2f} -> "
            f"rothead={new_rothead}, neck={new_neck}, "
            f"eye_y={desired_eye_y}, eye_x={desired_eye_x}"
        )

    @staticmethod
    def clamp(val: int, vmin: int, vmax: int) -> int:
        """Clamp val between vmin and vmax."""
        return max(vmin, min(val, vmax))

    @staticmethod
    def publish_servo(pub, angle: int):
        """Publish a servo angle command as an Int16 message."""
        msg = Int16()
        msg.data = angle
        pub.publish(msg)

def main(args=None):
    """
    Main entry point:
    - Initializes ROS 2 context
    - Creates and spins the FaceTrackingBehaviorNode
    - Handles shutdown on interrupt
    """
    rclpy.init(args=args)
    node = FaceTrackingBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
