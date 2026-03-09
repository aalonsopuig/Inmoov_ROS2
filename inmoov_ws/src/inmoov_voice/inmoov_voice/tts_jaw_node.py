#!/usr/bin/env python3
"""
tts_jaw_node.py

ROS 2 node that subscribes to /tts/say (std_msgs/String) and synthesizes incoming text using Piper.
It publishes jaw position commands on /jaw while playing audio with dynamic mouth movement.

Features:
- FIFO queue limited to 100 messages to avoid memory saturation
- Sequential processing of messages in a worker thread
- Larger analysis windows (50ms, 25ms hop)
- RMS energy mapped to three jaw levels: closed=5, medium=10, open=20
- Hysteresis thresholds to smooth transitions

Authors: Alejandro Alonso Puig + ChatGPT o4-mini-high
License: Apache 2.0
"""

import os                                # File path operations
import subprocess                        # Launch Piper binary
import threading                         # Threads for worker and timers
import queue                             # FIFO queue for incoming messages
import soundfile as sf                   # Read WAV file contents
import numpy as np                       # Numerical operations for energy calculation
import sounddevice as sd                 # Audio playback via default device

import rclpy                             # ROS 2 Python client library
from rclpy.node import Node              # Base class for ROS 2 nodes
from std_msgs.msg import String, Int16   # ROS 2 message types

class TTSJawNode(Node):
    def __init__(self):
        super().__init__('tts_jaw_node')
        # Directory where Piper binary and models reside
        self.model_dir = os.path.expanduser('~/piper')
        self.piper_bin = os.path.join(self.model_dir, 'piper')
        self.model_onnx = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx')
        self.model_cfg  = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx.json')
        self.output_wav = os.path.join(self.model_dir, 'output.wav')

        # FIFO queue for incoming text (max 100)
        self.queue = queue.Queue(maxsize=100)

        # Subscriber: incoming text
        self.subscription = self.create_subscription(
            String,
            '/tts/say',
            self.say_callback,
            10
        )
        self.get_logger().info('TTSJawNode ready: listening on /tts/say')

        # Publisher: jaw servo commands
        self.jaw_pub = self.create_publisher(Int16, '/jaw', 10)

        # Worker thread to process queue
        self.worker_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.worker_thread.start()

    def say_callback(self, msg: String):
        """Add incoming text to the queue if not full."""
        try:
            self.queue.put_nowait(msg.data)
            self.get_logger().info(f"Enqueued text: '{msg.data}'")
        except queue.Full:
            self.get_logger().warn("Queue full. Discarding message.")

    def _process_queue(self):
        """Worker loop to process queued messages sequentially."""
        while rclpy.ok():
            text = self.queue.get()  # Blocks until a message is available
            self._speak_and_move(text)
            self.queue.task_done()

    def _speak_and_move(self, text: str):
        """
        1) Generate WAV audio with Piper
        2) Analyze energy with 50ms frames, 25ms hop
        3) Apply hysteresis and map energy to jaw levels
        4) Play audio while publishing jaw movements
        """
        # Generate the WAV file via Piper
        proc = subprocess.Popen([
            self.piper_bin,
            '--model', self.model_onnx,
            '--config', self.model_cfg,
            '--output_file', self.output_wav
        ], stdin=subprocess.PIPE)
        proc.communicate(input=text.encode())
        self.get_logger().info('Audio file generated')

        # Read audio data
        data, sr = sf.read(self.output_wav)

        # Frame and hop for smoother control
        frame_len = int(0.05 * sr)   # 50ms frames
        hop_len   = int(0.025 * sr)  # 25ms hop

        # RMS energy per frame
        energies = []
        for start in range(0, len(data), hop_len):
            frame = data[start:start+frame_len]
            if len(frame) == 0:
                continue
            energies.append(np.sqrt(np.mean(frame**2)))
        energies = np.array(energies)

        # Hysteresis thresholds
        max_energy = energies.max() if energies.size > 0 else 1.0
        high_th = max_energy * 0.3   # upper threshold
        low_th  = max_energy * 0.1   # lower threshold

        # Map energies to jaw levels: closed=5, medium=10, open=20
        last_level = 5
        times = np.arange(len(energies)) * hop_len / sr
        for t, e in zip(times, energies):
            if e > high_th:
                level = 20
            elif e > low_th:
                level = 10
            else:
                level = 5

            # Only publish if level changed
            if level != last_level:
                threading.Timer(
                    t, lambda lvl=level: self.jaw_pub.publish(Int16(data=lvl))
                ).start()
                last_level = level

        # Ensure mouth opens at start
        self.jaw_pub.publish(Int16(data=20))
        self.get_logger().info('Jaw movement started')

        # Play audio
        sd.play(data, sr)
        sd.wait()
        self.get_logger().info('Audio playback completed')

        # Ensure closed at the end
        self.jaw_pub.publish(Int16(data=5))
        self.get_logger().info('Jaw closed')


def main(args=None):
    rclpy.init(args=args)
    node = TTSJawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

