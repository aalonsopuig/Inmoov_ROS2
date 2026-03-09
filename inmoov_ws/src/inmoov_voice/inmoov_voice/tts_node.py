#!/usr/bin/env python3
"""
tts_node.py

ROS 2 node that subscribes to /tts/say (std_msgs/String)
and synthesizes incoming text using Piper.
Implements a FIFO queue with a maximum size to prevent memory saturation.

Authors: Alejandro Alonso Puig + ChatGPT 5
License: Apache 2.0
"""

import os                       # File path handling
import subprocess               # Execute Piper and audio playback
import threading                # Worker thread and queue handling
import queue                    # FIFO queue for text buffering

import rclpy                    # ROS 2 Python client library
from rclpy.node import Node     # ROS 2 Node base class
from std_msgs.msg import String # Message type for incoming text


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # Directory where Piper binary and models are stored
        self.model_dir = os.path.expanduser('~/piper')
        self.piper_bin = os.path.join(self.model_dir, 'piper')
        self.model_onnx = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx')
        self.model_cfg  = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx.json')
        self.output_wav = os.path.join(self.model_dir, 'output.wav')

        # FIFO queue to store up to 100 pending texts
        self.queue = queue.Queue(maxsize=100)

        # Subscriber: listen for text to speak
        self.subscription = self.create_subscription(
            String,
            '/tts/say',
            self.say_callback,
            10
        )

        # Start background worker thread that processes the queue
        self.worker_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.worker_thread.start()

        self.get_logger().info('TTSNode ready: listening on /tts/say')

    def say_callback(self, msg: String):
        """
        Callback when a new message arrives on /tts/say.
        Adds the text to the queue unless it is full.
        """
        text = msg.data
        if self.queue.full():
            self.get_logger().warn("Queue full, discarding message")
        else:
            self.queue.put(text)
            self.get_logger().info(f"Enqueued text: '{text}'")

    def _process_queue(self):
        """
        Continuously processes queued texts one by one.
        Each text is synthesized and spoken before moving to the next.
        """
        while True:
            text = self.queue.get()  # Blocks until an item is available
            self._speak(text)
            self.queue.task_done()

    def _speak(self, text: str):
        """
        Generate audio with Piper and play it.
        """
        # 1) Generate WAV via Piper, feeding text to stdin
        proc = subprocess.Popen(
            [self.piper_bin,
             '--model', self.model_onnx,
             '--config', self.model_cfg,
             '--output_file', self.output_wav],
            stdin=subprocess.PIPE
        )
        proc.communicate(input=text.encode())
        self.get_logger().info(f"Audio generated for: '{text}'")

        # 2) Play the generated WAV
        subprocess.run(['aplay', self.output_wav], check=True)
        self.get_logger().info("Audio playback finished")


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
