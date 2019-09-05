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

import unittest
import rclpy

from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped, PointStamped


class TestBuffer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def build_transform(self, target, source, rclpy_time):
        transform = TransformStamped()
        transform.header.frame_id = target
        transform.header.stamp = rclpy_time.to_msg()
        transform.child_frame_id = source
        transform.transform.translation.x = 42.0
        transform.transform.translation.y = -3.14
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        return transform

    def test_can_transform_valid_transform(self):
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        self.assertEqual(buffer.set_transform(transform, 'unittest'), None)

        self.assertEqual(buffer.can_transform('foo', 'bar', rclpy_time), True)

        output = buffer.lookup_transform('foo', 'bar', rclpy_time)
        self.assertEqual(transform.child_frame_id, output.child_frame_id)
        self.assertEqual(transform.transform.translation.x, output.transform.translation.x)
        self.assertEqual(transform.transform.translation.y, output.transform.translation.y)
        self.assertEqual(transform.transform.translation.z, output.transform.translation.z)

    def test_await_transform_immediately_available(self):
        # wait for a transform that is already available to test short-cut code
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        buffer.set_transform(transform, 'unittest')

        coro = buffer.lookup_transform_async('foo', 'bar', rclpy_time)
        with self.assertRaises(StopIteration) as cm:
            coro.send(None)

        self.assertEqual(transform, cm.exception.value)
        coro.close()

    def test_await_transform_full_immediately_available(self):
        # wait for a transform that is already available to test short-cut code
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        buffer.set_transform(transform, 'unittest')

        coro = buffer.lookup_transform_full_async('foo', rclpy_time, 'bar', rclpy_time, 'foo')
        with self.assertRaises(StopIteration) as cm:
            coro.send(None)

        self.assertEqual(transform, cm.exception.value)
        coro.close()

    def test_await_transform_delayed(self):
        # wait for a transform that is not yet available
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        coro = buffer.lookup_transform_async('foo', 'bar', rclpy_time)
        coro.send(None)

        buffer.set_transform(transform, 'unittest')
        with self.assertRaises(StopIteration) as cm:
            coro.send(None)

        self.assertEqual(transform, cm.exception.value)
        coro.close()

    def test_await_transform_full_delayed(self):
        # wait for a transform that is not yet available
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        coro = buffer.lookup_transform_full_async('foo', rclpy_time, 'bar', rclpy_time, 'foo')
        coro.send(None)

        buffer.set_transform(transform, 'unittest')
        with self.assertRaises(StopIteration) as cm:
            coro.send(None)

        self.assertEqual(transform, cm.exception.value)
        coro.close()


if __name__ == '__main__':
    unittest.main()
