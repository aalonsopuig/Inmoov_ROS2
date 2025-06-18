#!/usr/bin/env python3
# face_recognized_behavior_node.py
#
# Author: Alejandro Alonso Puig (https://github.com/aalonsopuig) + ChatGPT 4.1
# License: Apache 2.0
#
# ROS 2 node for behavior when a specific person is recognized by the InMoov robot.
# Subscribes to /recognized_person (std_msgs/String) and executes actions:
#  - "none": do nothing
#  - "unknown": speak a random greeting from greetings_unknown.txt, rate-limited
#  - other names: speak personalized greeting from greetings_known.txt and
#    perform a gesture sequence from movements_known.yaml, rate-limited
# Publishes to:
#  - /tts/say (std_msgs/String) for speech synthesis by tts_jaw_node
#  - /<servo_topic> (std_msgs/Int16) for any servo described in the YAML

import os        # for locating package files
import time      # for rate-limiting behaviors
import random    # for random greeting selection
import yaml      # for parsing the YAML movement sequence file

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # base class for ROS 2 nodes
from std_msgs.msg import String  # for recognized_person and tts text
from std_msgs.msg import Int16   # for servo angle commands

class FaceRecognizedBehaviorNode(Node):
    """
    ROS 2 node that handles behavior when a person is recognized.
    Subscribes to /recognized_person, reads greeting templates from files,
    and publishes speech and arbitrary servo movements from a YAML sequence,
    rate-limited by a minimum interval.
    """
    BEHAVIOR_INTERVAL = 10.0  # seconds

    def __init__(self):
        """
        Initialize node, subscriptions, publishers, load resources, and state.
        """
        super().__init__('face_recognized_behavior_node')

        # Subscribe to recognized_person topic
        self.create_subscription(
            String,
            '/recognized_person',
            self.person_callback,
            10
        )

        # Publisher for text-to-speech
        self.tts_pub = self.create_publisher(String, '/tts/say', 10)
        # Dynamic dictionary of servo publishers
        self.servo_pubs = {}

        # Load greeting files and movement sequence
        pkg_dir = os.path.dirname(__file__)
        self.greetings_unknown = self._load_greetings(os.path.join(pkg_dir, 'greetings_unknown.txt'))
        self.greetings_known = self._load_greetings(os.path.join(pkg_dir, 'greetings_known.txt'))
        self.movement_sequence = self._load_movement_sequence(os.path.join(pkg_dir, 'movements_known.yaml'))

        # Timing control for behaviors
        self.last_behavior_time = 0.0

        # State for movement sequence execution
        self._move_steps = []
        self._move_index = 0
        self._move_timer = None

        self.get_logger().info('FaceRecognizedBehaviorNode started and ready.')

    def _load_greetings(self, filepath):
        """
        Read greeting lines from a text file, stripping whitespace and ignoring empty lines.
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                greetings = [line.strip() for line in f if line.strip()]
                self.get_logger().debug(f"Loaded {len(greetings)} greetings from {os.path.basename(filepath)}")
                return greetings
        except Exception as e:
            self.get_logger().error(f"Failed to load greetings from {filepath}: {e}")
            return []

    def _load_movement_sequence(self, filepath):
        """
        Load movement sequence from YAML file; each step has 'servos' map and 'delay'.
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                sequence = yaml.safe_load(f)
                count = len(sequence) if sequence else 0
                self.get_logger().debug(f"Loaded {count} movement steps from {os.path.basename(filepath)}")
                return sequence
        except Exception as e:
            self.get_logger().error(f"Failed to load movement sequence from {filepath}: {e}")
            return []

    def person_callback(self, msg: String):
        """
        Handle incoming recognized_person messages, triggering behaviors.
        """
        name = msg.data
        now = time.time()
        self.get_logger().debug(f"Received /recognized_person: '{name}'")

        # Rate limit: skip if interval not elapsed
        if now - self.last_behavior_time < self.BEHAVIOR_INTERVAL:
            self.get_logger().debug("Behavior rate-locked, skipping")
            return

        if name == 'none':
            # No action for 'none'
            return
        elif name == 'unknown':
            # Unknown face: random greeting
            if self.greetings_unknown:
                text = random.choice(self.greetings_unknown)
                self._publish_speech(text)
        else:
            # Known face: personalized greeting + movement sequence
            if self.greetings_known:
                template = random.choice(self.greetings_known)
                text = template.replace('<nombre>', name)
                self._publish_speech(text)
            self._start_movement_sequence()

        # Update last execution time
        self.last_behavior_time = now

    def _publish_speech(self, text: str):
        """
        Publish text to TTS topic /tts/say.
        """
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().debug(f"TTS: '{text}'")

    def _start_movement_sequence(self):
        """
        Initialize and start executing the gesture sequence loaded from YAML.
        """
        if not self.movement_sequence:
            self.get_logger().warn("No movement sequence loaded, cannot start sequence")
            return
        self.get_logger().debug("Starting movement sequence from movements_known.yaml")
        self._move_steps = self.movement_sequence.copy()
        self._move_index = 0
        self._run_next_step()

    def _run_next_step(self):
        """
        Execute current movement step: publish servo angles and schedule next via timer.
        """
        if self._move_index >= len(self._move_steps):
            self.get_logger().debug("Completed all movement steps.")
            self._move_steps = []
            self._move_index = 0
            return

        step = self._move_steps[self._move_index]
        servos = step.get('servos', {})
        # Publish each servo angle
        for topic, angle in servos.items():
            if topic not in self.servo_pubs:
                self.servo_pubs[topic] = self.create_publisher(Int16, f"/{topic}", 10)
            self.servo_pubs[topic].publish(Int16(data=int(angle)))
        delay = float(step.get('delay', 1.0))
        self.get_logger().debug(
            f"Step {self._move_index+1}/{len(self._move_steps)}: " +
            ", ".join([f"/{t}={a}" for t, a in servos.items()]) +
            f" | next step in {delay:.2f}s"
        )
        self._move_index += 1
        # Schedule next step without blocking
        self._move_timer = self.create_timer(delay, self._on_step_timer)

    def _on_step_timer(self):
        """
        Called when the delay timer expires; cancel timer and run next step.
        """
        if self._move_timer:
            self._move_timer.cancel()
            self._move_timer = None
        self._run_next_step()


def main(args=None):
    """
    Main entry point:
    Initialize ROS, spin node, handle shutdown.
    """
    rclpy.init(args=args)
    node = FaceRecognizedBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
