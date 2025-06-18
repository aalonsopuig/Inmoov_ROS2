#!/usr/bin/env python3
"""
tts_node.py

ROS 2 node that subscribes to /tts/say (std_msgs/String) 
and synthesizes incoming text using Piper,
Authors: Alejandro Alonso Puig + ChatGPT o4-mini-high
License: Apache 2.0
"""

import os                       # file path handling
import subprocess               # execute Piper and audio playback
import threading                # non-blocking speech execution

import rclpy                    # ROS 2 Python client library
from rclpy.node import Node     # ROS 2 Node base class
from std_msgs.msg import String # message type for incoming text

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')  # initialize node
        # directory where Piper binary and models are stored
        self.model_dir = os.path.expanduser('~/piper')
        self.piper_bin = os.path.join(self.model_dir, 'piper')
        self.model_onnx = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx')
        self.model_cfg  = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx.json')
        self.output_wav = os.path.join(self.model_dir, 'output.wav')

        # ROS subscriber: listen for text to speak
        self.subscription = self.create_subscription(
            String,
            '/tts/say',
            self.say_callback,
            10
        )
        self.get_logger().info('TTSNode ready: listening on /tts/say')


    def say_callback(self, msg: String):
        """
        Callback triggered when a String message arrives on /tts/say.
        Spawns a thread to handle speech.
        """
        text = msg.data
        self.get_logger().info(f"Received text: '{text}'")
        # Launch speech control in separate thread
        thread = threading.Thread(target=self._speak, args=(text,))
        thread.start()

    def _speak(self, text: str):
        """
        Generate audio with Piper, play it, and publish jaw open/close commands.
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
        self.get_logger().info('Audio file generated')

        # 2) Play the generated WAV
        subprocess.run(['aplay', self.output_wav], check=True)
        self.get_logger().info('Audio playback finished')


def main(args=None):
    rclpy.init(args=args)       # initialize ROS client library
    node = TTSNode()            # create node instance
    try:
        rclpy.spin(node)        # keep node alive to process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()     # clean up node resources
        rclpy.shutdown()        # shutdown ROS client library

if __name__ == '__main__':
    main()
