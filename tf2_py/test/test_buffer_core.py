# Copyright 2019 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import time
import unittest
import rclpy
from geometry_msgs.msg import TransformStamped
from test_tf2_py._tf2_py import BufferCore
from test_tf2_py._tf2_py import LookupException

def build_transform(target_frame, source_frame, stamp):
    transform = TransformStamped()
    transform.header.frame_id = target_frame
    transform.child_frame_id = source_frame
    transform.header.stamp = stamp
    transform.transform.translation.x = 2.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    return transform


class TestBufferClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def test_all_frames_as_yaml(self):
        buffer_core = BufferCore()

        transform = build_transform('bar', 'foo', rclpy.time.Time().to_msg())
        buffer_core.set_transform(transform, 'unittest')

        self.assertTrue(type(buffer_core.all_frames_as_yaml()) == str)
        self.assertTrue('foo' in buffer_core.all_frames_as_yaml())
        self.assertTrue('bar' in buffer_core.all_frames_as_yaml())

    def test_all_frames_as_string(self):
        buffer_core = BufferCore()

        transform = build_transform('bar', 'foo', rclpy.time.Time().to_msg())
        buffer_core.set_transform(transform, 'unittest')

        self.assertTrue(type(buffer_core.all_frames_as_string()) == str)
        self.assertTrue('foo' in buffer_core.all_frames_as_string())
        self.assertTrue('bar' in buffer_core.all_frames_as_string())

    def test_set_transform(self):
        buffer_core = BufferCore()
        transform = build_transform('bar', 'foo', rclpy.time.Time().to_msg())

        result = buffer_core.set_transform(transform, 'unittest')
        self.assertEqual(result, None)

        result, _ = buffer_core.can_transform_core(
            target_frame='bar',
            source_frame='foo',
            time=rclpy.time.Time(seconds=0.5)
        )
        self.assertEqual(result, 0)

    def test_set_transform_static(self):
        buffer_core = BufferCore()

        transform = build_transform('bar', 'foo', rclpy.time.Time().to_msg())

        result = buffer_core.set_transform_static(transform, 'unittest')
        self.assertEqual(result, None)

        result, _ = buffer_core.can_transform_core(
            target_frame='bar',
            source_frame='foo',
            time=rclpy.time.Time(seconds=0.5)
        )
        self.assertEqual(result, 1)

    def test_can_transform_core_pass(self):
        buffer_core = BufferCore()

        transform = build_transform('bar', 'foo', rclpy.time.Time().to_msg())

        buffer_core.set_transform(transform, 'unittest')

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=1).to_msg())

        buffer_core.set_transform(transform, 'unittest')

        result, error_msg = buffer_core.can_transform_core(
            target_frame='foo',
            source_frame='bar',
            time=rclpy.time.Time(seconds=0.5)
        )
        self.assertEqual(result, 1)
        self.assertEqual(error_msg, '')

    def test_can_transform_core_fail(self):
        buffer_core = BufferCore()

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=1).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=2).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        result, error_msg = buffer_core.can_transform_core(
            target_frame='foo',
            source_frame='bar',
            time=rclpy.time.Time(seconds=2.5)
        )
        self.assertEqual(result, 0)
        self.assertIn('extrapolation into the future', error_msg)

        result, error_msg = buffer_core.can_transform_core(
            target_frame='foo',
            source_frame='bar',
            time=rclpy.time.Time(seconds=0.5)
        )
        self.assertEqual(result, 0)
        self.assertIn('extrapolation into the past', error_msg)

    def test_can_transform_full_core(self):
        buffer_core = BufferCore()

        transform = build_transform('bar', 'foo', rclpy.time.Time().to_msg())
        buffer_core.set_transform(transform, 'unittest')

        transform = build_transform('foo', 'baz', rclpy.time.Time().to_msg())
        buffer_core.set_transform(transform, 'unittest')

        result, error_msg = buffer_core.can_transform_full_core(
            target_frame='foo',
            target_time=rclpy.time.Time(),
            source_frame='baz',
            source_time=rclpy.time.Time(),
            fixed_frame='bar'
        )

        self.assertEqual(result, 1)
        self.assertEqual(error_msg, '')

    def test_get_latest_common_time(self):
        buffer_core = BufferCore()

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=0).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=1).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=0.5).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        latest_common_time = buffer_core.get_latest_common_time('bar', 'foo')

        self.assertEqual(latest_common_time, rclpy.time.Time(seconds=1))

    def test_clear(self):
        buffer_core = BufferCore()

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=0).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        result, _ = buffer_core.can_transform_core(
            target_frame='foo',
            source_frame='bar',
            time=rclpy.time.Time()
        )
        self.assertTrue(result)

        buffer_core.clear()

        result, _ = buffer_core.can_transform_core(
            target_frame='foo',
            source_frame='bar',
            time=rclpy.time.Time()
        )
        self.assertFalse(result)

    def test_lookup_transform_core_pass(self):
        buffer_core = BufferCore()

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=0).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        lookup_transform = buffer_core.lookup_transform_core(
            target_frame='bar',
            source_frame='foo',
            time=rclpy.time.Time()
        )

        self.assertEqual(transform, lookup_transform)

    def test_lookup_transform_core_fail(self):
        buffer_core = BufferCore()

        transform = build_transform(
            'bar', 'foo', rclpy.time.Time(seconds=0).to_msg())
        buffer_core.set_transform(transform, 'unittest')

        with self.assertRaises(LookupException) as ex:
            buffer_core.lookup_transform_core(
                target_frame='bar',
                source_frame='baz',
                time=rclpy.time.Time()
            )

        self.assertEqual(LookupException, type(ex.exception))

if __name__ == '__main__':
    unittest.main()
