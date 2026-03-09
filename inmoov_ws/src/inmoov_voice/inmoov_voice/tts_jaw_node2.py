#!/usr/bin/env python3
"""
tts_jaw_node.py

ROS 2 node that subscribes to /tts/say (std_msgs/String) and synthesizes incoming text using Piper,
publishing jaw open/close commands on /jaw and playing the resulting audio with dynamic mouth movement.
Authors: Alejandro Alonso Puig + ChatGPT o4-mini-high
License: Apache 2.0
"""

import os                                # File path operations
import subprocess                        # Launch Piper binary
import threading                         # Schedule non-blocking tasks
import soundfile as sf                   # Read WAV file contents
import numpy as np                       # Numerical operations for energy calculation
import sounddevice as sd                 # Audio playback via default device

import rclpy                             # ROS 2 Python client library
from rclpy.node import Node              # Base class for ROS 2 nodes
from std_msgs.msg import String, Int16   # Message types: incoming text and jaw commands

class TTSJawNode(Node):
    def __init__(self):
        super().__init__('tts_jaw_node')  # update node name
        # Directory where Piper binary and models reside
        self.model_dir = os.path.expanduser('~/piper')
        self.piper_bin = os.path.join(self.model_dir, 'piper')
        self.model_onnx = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx')
        self.model_cfg  = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx.json')
        self.output_wav = os.path.join(self.model_dir, 'output.wav')

        # Subscriber: listen for incoming text to synthesize
        self.subscription = self.create_subscription(
            String,
            '/tts/say',
            self.say_callback,
            10
        )
        self.get_logger().info('TTSJawNode ready: listening on /tts/say')

        # Publisher: send jaw position commands (10=closed, 20=open)
        self.jaw_pub = self.create_publisher(Int16, '/jaw', 10)

    def say_callback(self, msg: String):
        """
        Called when a message arrives on /tts/say.
        Spawns thread to handle speech synthesis and dynamic jaw movement.
        """
        text = msg.data
        self.get_logger().info(f"Received text: '{text}'")
        thread = threading.Thread(target=self._speak_and_move, args=(text,))
        thread.start()

    def _speak_and_move(self, text: str):
        """
        1) Invoke Piper to generate WAV.
        2) Analyze WAV energy to find voiced intervals.
        3) Schedule jaw open/close at voiced/silence boundaries.
        4) Play audio and finalize mouth closed.
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

        # Read audio data and sampling rate
        data, sr = sf.read(self.output_wav)  # data: np.ndarray, sr: int

        # Compute frame/ hop lengths for 20ms frames, 10ms hop
        frame_len = int(0.02 * sr)
        hop_len   = int(0.01 * sr)

        # Calculate RMS energy per frame
        energies = []
        for start in range(0, len(data), hop_len):
            frame = data[start:start+frame_len]
            energies.append(np.sqrt(np.mean(frame**2)))
        energies = np.array(energies)

        # Determine voiced intervals via threshold (10% of max energy)
        threshold = energies.max() * 0.1
        voiced = energies > threshold
        changes = np.diff(voiced.astype(int))
        opens  = np.where(changes == 1)[0]
        closes = np.where(changes == -1)[0]
        times_open  = opens  * hop_len / sr
        times_close = closes * hop_len / sr

        # Schedule jaw open/close using threading.Timer
        for t in times_open:
            threading.Timer(t, lambda: self.jaw_pub.publish(Int16(data=20))).start()
        for t in times_close:
            threading.Timer(t, lambda: self.jaw_pub.publish(Int16(data=10))).start()

        # Ensure mouth open at start
        self.jaw_pub.publish(Int16(data=20))
        self.get_logger().info('Jaw opened')

        # Play audio in real time
        sd.play(data, sr)
        sd.wait()
        self.get_logger().info('Audio playback completed')

        # Close mouth at end
        self.jaw_pub.publish(Int16(data=10))
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

