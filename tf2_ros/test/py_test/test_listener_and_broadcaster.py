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

from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros import ExtrapolationException


def build_transform(target_frame, source_frame, stamp):
    transform = TransformStamped()
    transform.header.frame_id = target_frame
    transform.header.stamp = stamp
    transform.child_frame_id = source_frame

    transform.transform.translation.x = 42.0
    transform.transform.translation.y = -3.14
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0

    return transform


class TestBroadcasterAndListener(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.buffer = Buffer()
        cls.node = rclpy.create_node('test')
        cls.broadcaster = TransformBroadcaster(cls.node)
        cls.static_broadcaster = StaticTransformBroadcaster(cls.node)
        cls.listener = TransformListener(
            buffer=cls.buffer, node=cls.node, spin_thread=False)

        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        pass

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        pass

    def setUp(self):
        self.buffer = Buffer()
        self.listener.buffer = self.buffer

    def broadcast_transform(self, target_frame, source_frame, time_stamp):
        broadcast_transform = build_transform(
            target_frame=target_frame, source_frame=source_frame, stamp=time_stamp)

        self.broadcaster.sendTransform(broadcast_transform)

        self.executor.spin_once()

        return broadcast_transform

    def broadcast_static_transform(self, target_frame, source_frame, time_stamp):
        broadcast_transform = build_transform(
            target_frame=target_frame, source_frame=source_frame, stamp=time_stamp)

        self.static_broadcaster.sendTransform(broadcast_transform)

        self.executor.spin_once()

        return broadcast_transform

    def test_broadcaster_and_listener(self):
        time_stamp = rclpy.time.Time(seconds=1, nanoseconds=0).to_msg()

        broadcasted_transform = self.broadcast_transform(
            target_frame='foo', source_frame='bar', time_stamp=time_stamp)

        listened_transform = self.buffer.lookup_transform(
            target_frame='foo', source_frame='bar', time=time_stamp)

        self.assertTrue(broadcasted_transform, listened_transform)

    def test_extrapolation_exception(self):
        self.broadcast_transform(
            target_frame='foo', source_frame='bar',
            time_stamp=rclpy.time.Time(seconds=0.3, nanoseconds=0).to_msg())

        self.broadcast_transform(
            target_frame='foo', source_frame='bar',
            time_stamp=rclpy.time.Time(seconds=0.2, nanoseconds=0).to_msg())

        with self.assertRaises(ExtrapolationException) as e:
            self.buffer.lookup_transform(
                target_frame='foo', source_frame='bar',
                time=rclpy.time.Time(seconds=0.1, nanoseconds=0).to_msg())

        self.assertTrue(
            'Lookup would require extrapolation into the past' in str(e.exception))

        with self.assertRaises(ExtrapolationException) as e:
            self.buffer.lookup_transform(
                target_frame='foo', source_frame='bar',
                time=rclpy.time.Time(seconds=0.4, nanoseconds=0).to_msg())

        self.assertTrue(
            'Lookup would require extrapolation into the future' in str(e.exception))

    def test_static_broadcaster_and_listener(self):
        broadcasted_transform = self.broadcast_static_transform(
            target_frame='foo', source_frame='bar',
            time_stamp=rclpy.time.Time(seconds=1.1, nanoseconds=0).to_msg())

        listened_transform = self.buffer.lookup_transform(
            target_frame='foo', source_frame='bar',
            time=rclpy.time.Time(seconds=1.5, nanoseconds=0).to_msg())

        self.assertTrue(broadcasted_transform, listened_transform)


if __name__ == '__main__':
    unittest.main()
