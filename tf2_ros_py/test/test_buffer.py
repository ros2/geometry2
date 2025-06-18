# Copyright (c) 2019 Open Source Robotics Foundation, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
import rclpy.executors
import rclpy.task
import tf2_ros as tf2
from tf2_ros.buffer import Buffer


class TestBuffer:

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

        assert buffer.set_transform(transform, 'unittest') is None
        assert buffer.can_transform('foo', 'bar', rclpy_time)

        output = buffer.lookup_transform('foo', 'bar', rclpy_time)

        assert transform.child_frame_id == output.child_frame_id
        assert transform.transform.translation.x == output.transform.translation.x
        assert transform.transform.translation.y == output.transform.translation.y
        assert transform.transform.translation.z == output.transform.translation.z

    def test_await_transform_immediately_available(self):
        # wait for a transform that is already available to test short-cut code
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        buffer.set_transform(transform, 'unittest')

        coro = buffer.lookup_transform_async('foo', 'bar', rclpy_time)
        with pytest.raises(StopIteration) as excinfo:
            coro.send(None)

        assert transform == excinfo.value.value
        coro.close()

    def test_await_transform_full_immediately_available(self):
        # wait for a transform that is already available to test short-cut code
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        buffer.set_transform(transform, 'unittest')

        coro = buffer.lookup_transform_full_async('foo', rclpy_time, 'bar', rclpy_time, 'foo')
        with pytest.raises(StopIteration) as excinfo:
            coro.send(None)

        assert transform == excinfo.value.value
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
        with pytest.raises(StopIteration) as excinfo:
            coro.send(None)

        assert transform == excinfo.value.value
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
        with pytest.raises(StopIteration) as excinfo:
            coro.send(None)

        assert transform == excinfo.value.value
        coro.close()


    def await_transform_timeout_template(self, transform_coroutine):
        # wait for timeout in an async call

        # We need a node environment to have an event loop with timers running.
        context = rclpy.context.Context()
        rclpy.init(context=context)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        node = rclpy.create_node('test_buffer', context=context)
        buffer = Buffer(node=node)
        current_time = node.get_clock().now()
        stop_fut = rclpy.task.Future()

        async def async_environment():
            try:
                # This allows us to test both the await_transform and await_transform_full
                await transform_coroutine(buffer, 'foo', 'bar', current_time)

                # Indicate that we had a success
                stop_fut.set_result(True)
            except tf2.LookupException as e:
                # Indicate that we had a timeout
                stop_fut.set_result(False)

        # This is a workaround to run the wait in the event loop
        gc = node.create_guard_condition(async_environment)

        # Trigger the guard condition to start the async call
        gc.trigger()

        # Runs the event loop until the future is done or times out (not the timeout that we want to test)
        rclpy.spin_until_future_complete(node, stop_fut, executor, timeout_sec=0.2)

        # Check if we actually timed out
        assert stop_fut.done() and not stop_fut.result()

        # Add a transform to the buffer to ensure that the buffer is still functional
        transform = self.build_transform('foo', 'bar', current_time)
        buffer.set_transform(transform, 'unittest')

        # Refresh the future to check if we can still use the buffer
        stop_fut = rclpy.task.Future()

        # Trigger the guard condition again to ensure that the buffer can still be used
        gc.trigger()

        # Run the event loop again
        rclpy.spin_until_future_complete(node, stop_fut, executor, timeout_sec=0.2)

        # Check if we can still get the transform after the timeout
        assert stop_fut.done() and stop_fut.result()


    def test_await_transform_timeout(self):
        # wait for timeout in an async call

        def transform_coroutine(buffer, target, source, rclpy_time):
            return buffer.wait_for_transform_async(target, source, rclpy_time, timeout=rclpy.duration.Duration(seconds=0.1))

        self.await_transform_timeout_template(transform_coroutine)

    def test_await_transform_full_timeout(self):
        # wait for timeout in an async call

        def transform_coroutine(buffer, target, source, rclpy_time):
            return buffer.wait_for_transform_full_async(target, rclpy_time, source, rclpy_time, target, timeout=rclpy.duration.Duration(seconds=0.1))

        self.await_transform_timeout_template(transform_coroutine)

    def test_async_lookup_transform_timeout(self):
        # wait for timeout in an async call

        def transform_coroutine(buffer, target, source, rclpy_time):
            return buffer.lookup_transform_async(target, source, rclpy_time, timeout=rclpy.duration.Duration(seconds=0.1))

        self.await_transform_timeout_template(transform_coroutine)

    def test_async_lookup_transform_full_timeout(self):
        # wait for timeout in an async call

        def transform_coroutine(buffer, target, source, rclpy_time):
            return buffer.lookup_transform_full_async(target, rclpy_time, source, rclpy_time, target, timeout=rclpy.duration.Duration(seconds=0.1))

        self.await_transform_timeout_template(transform_coroutine)

    def test_buffer_non_default_cache(self):
        buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        transform = self.build_transform('foo', 'bar', rclpy_time)

        assert buffer.set_transform(transform, 'unittest') is None

        assert buffer.can_transform('foo', 'bar', rclpy_time)

        output = buffer.lookup_transform('foo', 'bar', rclpy_time)
        assert transform.child_frame_id == output.child_frame_id
        assert transform.transform.translation.x == output.transform.translation.x
        assert transform.transform.translation.y == output.transform.translation.y
        assert transform.transform.translation.z == output.transform.translation.z
