#!/usr/bin/env python3
# reading_tool.py
#
# Author: Alejandro Alonso Puig (https://github.com/aalonsopuig) + ChatGPT 5
# License: Apache 2.0
#
# Utility program that reads text from text_to_read.txt and publishes
# each line to /tts/say with a pause between lines. Once all
# lines are published, the program exits.

import os        # for locating the text file
import time      # for pause between lines
import rclpy     # ROS 2 client library
from rclpy.node import Node
from std_msgs.msg import String  # message type for TTS

class ReadingUtility(Node):
    """
    ROS 2 utility node that reads a text file line by line,
    publishes each line to /tts/say, waits, and exits when done.
    """

    FILE_NAME = 'text_to_read.txt'   # file name to read
    PAUSE_SECONDS = 10.0              # pause between lines

    def __init__(self):
        super().__init__('reading_behavior_node')

        # Publisher to /tts/say
        self.tts_pub = self.create_publisher(String, '/tts/say', 10)

        # Path to the file (same folder as script)
        pkg_dir = os.path.dirname(__file__)
        filepath = os.path.join(pkg_dir, self.FILE_NAME)

        # Read file into memory
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                self.lines = [line.strip() for line in f if line.strip()]
        except Exception as e:
            self.get_logger().error(f"Failed to read {filepath}: {e}")
            self.lines = []

    def run(self):
        """Publish each line with a pause, then exit."""
        if not self.lines:
            self.get_logger().warn("No lines to read.")
            return

        self.get_logger().info(f"Starting to read {len(self.lines)} lines...")

        for i, text in enumerate(self.lines, start=1):
            msg = String()
            msg.data = text
            self.tts_pub.publish(msg)
            self.get_logger().info(f"Line {i}: {text}")
            time.sleep(self.PAUSE_SECONDS)

        self.get_logger().info("Finished reading all lines.")


def main(args=None):
    rclpy.init(args=args)
    node = ReadingUtility()
    node.run()            # sequential reading
    node.destroy_node()   # cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main()
