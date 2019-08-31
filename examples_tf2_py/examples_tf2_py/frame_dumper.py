#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameDumper(Node):
    """Print all frames to stdout at a regular interval."""

    def __init__(self):
        super().__init__('example_frame_dumper')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._output_timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        print(self._tf_buffer.all_frames_as_yaml())


def main():
    rclpy.init()
    node = FrameDumper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
