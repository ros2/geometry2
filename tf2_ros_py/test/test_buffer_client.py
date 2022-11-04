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

import pytest
import threading
import time

import rclpy

from tf2_ros.buffer_client import BufferClient
from geometry_msgs.msg import TransformStamped
from tf2_msgs.action import LookupTransform
from tf2_py import BufferCore, LookupException
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from tf2_msgs.msg import TF2Error


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


class MockBufferServer():
    def __init__(self, node, buffer_core):
        self.action_server = rclpy.action.ActionServer(node, LookupTransform, 'lookup_transform', self.execute_callback)
        self.buffer_core = buffer_core

    def execute_callback(self, goal_handle):
        response = LookupTransform.Result()
        response.transform = TransformStamped()
        response.error = TF2Error()

        try:
            if not goal_handle.request.advanced:
                transform = self.buffer_core.lookup_transform_core(target_frame=goal_handle.request.target_frame,
                                                                   source_frame=goal_handle.request.source_frame,
                                                                   time=Time.from_msg(goal_handle.request.source_time))
            else:
                transform = self.buffer_core.lookup_transform_full_core(
                    target_frame=goal_handle.request.target_frame,
                    target_time=Time.from_msg(goal_handle.request.target_time),
                    source_frame=goal_handle.request.source_frame,
                    source_time=Time.from_msg(goal_handle.request.source_time),
                    fixed_frame=goal_handle.request.fixed_frame
                )
            response.transform = transform
            goal_handle.succeed()
        except LookupException as e:
            response.error.error = TF2Error.LOOKUP_ERROR
            goal_handle.abort()

        return response

    def destroy(self):
        self.action_server.destroy()


class TestBufferClient:
    @classmethod
    def setup_class(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestBufferClient', context=cls.context)
        cls.executor.add_node(cls.node)

        buffer_core = BufferCore()
        transform = build_transform('foo', 'bar', rclpy.time.Time().to_msg())
        buffer_core.set_transform(transform, 'unittest')

        cls.mock_action_server = MockBufferServer(cls.node, buffer_core)

    @classmethod
    def teardown_class(cls):
        cls.mock_action_server.destroy()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setup_method(self, method):
        self.spinning = threading.Event()
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()

    def teardown_method(self, method):
        self.spinning.set()
        self.spin_thread.join()

    def spin(self):
        while self.context.ok() and not self.spinning.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def test_lookup_transform_true(self):
        buffer_client = BufferClient(
            self.node, 'lookup_transform', check_frequency=10.0, timeout_padding=rclpy.duration.Duration(seconds=0.0))

        result = buffer_client.lookup_transform(
            'foo', 'bar', rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))

        tf = build_transform('foo', 'bar', rclpy.time.Time().to_msg())
        assert tf == result
        buffer_client.destroy()

    def test_lookup_transform_fail(self):
        buffer_client = BufferClient(
            self.node, 'lookup_transform', check_frequency=10.0, timeout_padding=rclpy.duration.Duration(seconds=0.0))

        with pytest.raises(LookupException) as excinfo:
            buffer_client.lookup_transform(
                'bar', 'baz', rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))

        assert LookupException == excinfo.type

        buffer_client.destroy()
