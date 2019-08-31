#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class WaitsForTransform(Node):
    """
    Wait for a transform syncronously.

    This class is an example of waiting for transforms.
    This will block the executor if used within a callback.
    See the async example showing how to use coroutine callbacks to avoid this.
    """

    def __init__(self):
        super().__init__('example_waits_for_transform')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._output_timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        from_frame = 'sonar_2'
        to_frame = 'starboard_wheel'
        self.get_logger().info('Waiting for transform from {} to {}'.format(from_frame, to_frame))
        try:
            # Block until the transform is available
            when = rclpy.time.Time()
            self._tf_buffer.lookup_transform(
                to_frame, from_frame, when, timeout=Duration(seconds=5.0))
        except LookupException:
            self.get_logger().info('transform not ready')
        else:
            self.get_logger().info('Got transform')


def main():
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    node = WaitsForTransform()
    # this node blocks in a callback, so a MultiThreadedExecutor is required
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    rclpy.shutdown()

