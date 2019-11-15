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

import threading

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class WaitsForTransform(Node):
    """
    Wait for a transform using callbacks.

    This class is an example of waiting for transforms.
    It avoids blocking the executor by registering a callback to be called when a transform becomes
    available.
    """

    def __init__(self):
        super().__init__('example_waits_for_transform')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._output_timer = self.create_timer(1.0, self.on_timer)

        self._lock = threading.Lock()
        self._tf_future = None
        self._from_frame = None
        self._to_frame = None
        self._when = None

    def on_tf_ready(self, future):
        with self._lock:
            self._tf_future = None
            if future.result():
                try:
                    self._tf_buffer.lookup_transform(self._to_frame, self._from_frame, self._when)
                except LookupException:
                    self.get_logger().info('transform no longer available')
                else:
                    self.get_logger().info('Got transform')

    def on_timer(self):
        if self._tf_future:
            self.get_logger().warn('Still waiting for transform')
            return

        with self._lock:
            self._from_frame = 'sonar_2'
            self._to_frame = 'starboard_wheel'
            self._when = rclpy.time.Time()
            self._tf_future = self._tf_buffer.wait_for_transform_async(
                self._to_frame, self._from_frame, self._when)
            self._tf_future.add_done_callback(self.on_tf_ready)
            self.get_logger().info('Waiting for transform from {} to {}'.format(
                self._from_frame, self._to_frame))


def main():
    rclpy.init()
    node = WaitsForTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
