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


class AsyncWaitsForTransform(Node):
    """
    Wait for a transform asyncronously using coroutines.

    This class is an example of waiting for transforms inside a callback
    without blocking the executor.
    The timer callback is a coroutine that gets suspended until the transform
    becomes available.
    It is assumed the TransformListener subscriptions are handled by the same
    executor as the timer callback, which this class does by creating both on
    the same node.
    """

    def __init__(self):
        super().__init__('example_async_waits_for_transform')

        # Create
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Try to get the transform every second.
        # If the transform is unavailable the timer callback will wait for it.
        self._output_timer = self.create_timer(1.0, self.on_timer)

    async def on_timer(self):
        from_frame = 'sonar_2'
        to_frame = 'starboard_wheel'

        self.get_logger().info('Waiting for transform from {} to {}'.format(from_frame, to_frame))

        # Get latest available
        when = rclpy.time.Time()

        try:
            # Suspends callback until transform becomes available
            transform = await self._tf_buffer.lookup_transform_async(to_frame, from_frame, when)
            self.get_logger().info('Got {}'.format(repr(transform)))
        except LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))


def main():
    rclpy.init()
    node = AsyncWaitsForTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

