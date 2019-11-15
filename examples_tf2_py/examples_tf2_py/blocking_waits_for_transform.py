# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class BlockingWaitsForTransform(Node):
    """
    Wait for a transform syncronously.

    This class is an example of waiting for transforms.
    This will block the executor if used within a callback.
    Coroutine callbacks should be used instead to avoid this.
    See :doc:`examples_tf2_py/async_waits_for_transform.py` for an example.
    """

    def __init__(self):
        super().__init__('example_blocking_waits_for_transform')

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
    node = BlockingWaitsForTransform()
    # this node blocks in a callback, so a MultiThreadedExecutor is required
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    rclpy.shutdown()
