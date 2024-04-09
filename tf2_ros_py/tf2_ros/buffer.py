# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Wim Meeussen
from typing import TypeVar
from typing import Optional
from typing import Any
from typing import List
from typing import Tuple
from typing import Callable

import threading

import rclpy
import tf2_py as tf2
import tf2_ros
from tf2_msgs.srv import FrameGraph
from geometry_msgs.msg import TransformStamped
# TODO(vinnamkim): It seems rosgraph is not ready
# import rosgraph.masterapi
from time import sleep
from rclpy.clock import JumpThreshold, TimeJump
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.task import Future

FrameGraphSrvRequest = TypeVar('FrameGraphSrvRequest')
FrameGraphSrvResponse = TypeVar('FrameGraphSrvResponse')


class Buffer(tf2.BufferCore, tf2_ros.BufferInterface):
    """
    Standard implementation of the :class:`tf2_ros.BufferInterface` abstract data type.

    Inherits from :class:`tf2_ros.buffer_interface.BufferInterface` and :class:`tf2.BufferCore`.

    Stores known frames and offers a ROS service, "tf_frames", which responds to client requests
    with a response containing a :class:`tf2_msgs.FrameGraph` representing the relationship of
    known frames.
    """

    def __init__(
        self,
        cache_time: Optional[Duration] = None,
        node: Optional[Node] = None
    ) -> None:
        """
        Constructor.

        :param cache_time: Duration object describing how long to retain past information in BufferCore.
        :param node: Create a tf2_frames service that returns all frames as a yaml document.
        """
        if cache_time is not None:
            tf2.BufferCore.__init__(self, cache_time)
        else:
            tf2.BufferCore.__init__(self)
        tf2_ros.BufferInterface.__init__(self)

        self._new_data_callbacks: List[Callable[[], None]] = []
        self._callbacks_to_remove: List[Callable[[], None]] = []
        self._callbacks_lock = threading.RLock()

        if node is not None:
            self.srv = node.create_service(FrameGraph, 'tf2_frames', self.__get_frames)
            self.clock = node.get_clock()
        else:
            self.clock = rclpy.clock.Clock()

        # create a jump callback so as to clear the buffer if use_sim_true is true and there is a jump in time
        threshold = JumpThreshold(min_forward=None,
                                  min_backward=Duration(seconds=-1),
                                  on_clock_change=True)
        self.jump_handle = self.clock.create_jump_callback(threshold, post_callback=self.time_jump_callback)

    def __get_frames(
        self,
        req: FrameGraphSrvRequest,
        res: FrameGraphSrvResponse
    ) -> FrameGraphSrvResponse:
        return FrameGraph.Response(frame_yaml=self.all_frames_as_yaml())

    def time_jump_callback(time_jump: TimeJump):
        rclpy.logging.get_logger("tf2_buffer").warning("Detected jump back in time. Clearing tf buffer.")
        self.clear()

    def set_transform(self, transform: TransformStamped, authority: str) -> None:
        super().set_transform(transform, authority)
        self._call_new_data_callbacks()

    def set_transform_static(self, transform: TransformStamped, authority: str) -> None:
        super().set_transform_static(transform, authority)
        self._call_new_data_callbacks()

    def _call_new_data_callbacks(self) -> None:
        with self._callbacks_lock:
            for callback in self._new_data_callbacks:
                callback()
            # Remove callbacks after to avoid modifying list being iterated on
            for callback in self._callbacks_to_remove:
                self._new_data_callbacks.remove(callback)
            self._callbacks_to_remove.clear()

    def _remove_callback(self, callback: Callable[[], None]) -> None:
        with self._callbacks_lock:
            # Actually remove the callback later
            self._callbacks_to_remove.append(callback)

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Duration = Duration()
    ) -> TransformStamped:
        """
        Get the transform from the source frame to the target frame.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform (0 will get the latest).
        :param timeout: Time to wait for the target frame to become available.
        :return: The transform between the frames.
        """
        self.can_transform(target_frame, source_frame, time, timeout)
        return self.lookup_transform_core(target_frame, source_frame, time)

    async def lookup_transform_async(
        self,
        target_frame: str,
        source_frame: str,
        time: Time
    ) -> TransformStamped:
        """
        Get the transform from the source frame to the target frame asyncronously.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform (0 will get the latest).
        :return: The transform between the frames.
        """
        await self.wait_for_transform_async(target_frame, source_frame, time)
        return self.lookup_transform_core(target_frame, source_frame, time)

    def lookup_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Duration = Duration()
    ) -> TransformStamped:
        """
        Get the transform from the source frame to the target frame using the advanced API.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to (0 will get the latest).
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated (0 will get the latest).
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: Time to wait for the target frame to become available.
        :return: The transform between the frames.
        """
        self.can_transform_full(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)
        return self.lookup_transform_full_core(
          target_frame, target_time, source_frame, source_time, fixed_frame)

    async def lookup_transform_full_async(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str
    ) -> TransformStamped:
        """
        Get the transform from the source frame to the target frame using the advanced API asyncronously.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to (0 will get the latest).
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated (0 will get the latest).
        :param fixed_frame: Name of the frame to consider constant in time.
        :return: The transform between the frames.
        """
        await self.wait_for_transform_full_async(target_frame, target_time, source_frame, source_time, fixed_frame)
        return self.lookup_transform_full_core(
            target_frame, target_time, source_frame, source_time, fixed_frame)

    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Duration = Duration(),
        return_debug_tuple: bool = False
    ) -> bool:
        """
        Check if a transform from the source frame to the target frame is possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform (0 will get the latest).
        :param timeout: Time to wait for the target frame to become available.
        :param return_debug_type: If true, return a tuple representing debug information.
        :return: The information of the transform being waited on.
        """
        clock = rclpy.clock.Clock()
        if timeout != Duration():
            start_time = clock.now()
            while (clock.now() < start_time + timeout and
                   not self.can_transform_core(target_frame, source_frame, time)[0] and
                   (clock.now() + Duration(seconds=3.0)) >= start_time): # big jumps in time are likely bag loops, so break for them
                # TODO(Anyone): We can't use Rate here because it would never expire
                # with a single-threaded executor.
                # See https://github.com/ros2/geometry2/issues/327 for ideas on
                # how to timeout waiting for transforms that don't block the executor.
                sleep(0.02)

        core_result = self.can_transform_core(target_frame, source_frame, time)
        if return_debug_tuple:
            return core_result
        return core_result[0]

    def can_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Duration = Duration(),
        return_debug_tuple: bool = False
    ) -> bool:
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to (0 will get the latest).
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated (0 will get the latest).
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: Time to wait for the target frame to become available.
        :param return_debug_type: If true, return a tuple representing debug information.
        :return: The information of the transform being waited on.
        """
        clock = rclpy.clock.Clock()
        if timeout != Duration():
            start_time = clock.now()
            while (clock.now() < start_time + timeout and
                   not self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)[0] and
                   (clock.now() + Duration(seconds=3.0)) >= start_time): # big jumps in time are likely bag loops, so break for them
                # TODO(Anyone): We can't use Rate here because it would never expire
                # with a single-threaded executor.
                # See https://github.com/ros2/geometry2/issues/327 for ideas on
                # how to timeout waiting for transforms that don't block the executor.
                sleep(0.02)
        core_result = self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)
        if return_debug_tuple:
            return core_result
        return core_result[0]

    def wait_for_transform_async(
        self,
        target_frame: str,
        source_frame: str,
        time: Time
    ) -> Future:
        """
        Wait for a transform from the source frame to the target frame to become possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform (0 will get the latest).
        :return: A future that becomes true when the transform is available.
        """
        fut = rclpy.task.Future()
        if self.can_transform_core(target_frame, source_frame, time)[0]:
            # Short cut, the transform is available
            fut.set_result(self.lookup_transform(target_frame, source_frame, time))
            return fut

        def _on_new_data():
            try:
                if self.can_transform_core(target_frame, source_frame, time)[0]:
                    fut.set_result(self.lookup_transform(target_frame, source_frame, time))
            except BaseException as e:
                fut.set_exception(e)

        self._new_data_callbacks.append(_on_new_data)
        fut.add_done_callback(lambda _: self._remove_callback(_on_new_data))

        return fut

    def wait_for_transform_full_async(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str
    ) -> Future:
        """
        Wait for a transform from the source frame to the target frame to become possible.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to (0 will get the latest).
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated (0 will get the latest).
        :param fixed_frame: Name of the frame to consider constant in time.
        :return: A future that becomes true when the transform is available.
        """
        fut = rclpy.task.Future()
        if self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)[0]:
            # Short cut, the transform is available
            fut.set_result(self.lookup_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame))
            return fut

        def _on_new_data():
            try:
                if self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)[0]:
                    fut.set_result(self.lookup_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame))
            except BaseException as e:
                fut.set_exception(e)

        self._new_data_callbacks.append(_on_new_data)
        fut.add_done_callback(lambda _: self._remove_callback(_on_new_data))

        return fut
