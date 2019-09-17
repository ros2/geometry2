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

import threading

import rclpy
import tf2_py as tf2
import tf2_ros
from tf2_msgs.srv import FrameGraph
# TODO(vinnamkim): It seems rosgraph is not ready
# import rosgraph.masterapi
from time import sleep
from rclpy.duration import Duration
from rclpy.task import Future


class Buffer(tf2.BufferCore, tf2_ros.BufferInterface):
    """
    Standard implementation of the :class:`tf2_ros.BufferInterface` abstract data type.

    Inherits from :class:`tf2_ros.buffer_interface.BufferInterface` and :class:`tf2.BufferCore`.

    Stores known frames and offers a ROS service, "tf_frames", which responds to client requests
    with a response containing a :class:`tf2_msgs.FrameGraph` representing the relationship of
    known frames. 
    """

    def __init__(self, cache_time = None):
        """
        Constructor.

        :param cache_time: (Optional) How long to retain past information in BufferCore.
        """
        if cache_time != None:
            tf2.BufferCore.__init__(self, cache_time)
        else:
            tf2.BufferCore.__init__(self)
        tf2_ros.BufferInterface.__init__(self)

        self._new_data_callbacks = []
        self._callbacks_to_remove = []
        self._callbacks_lock = threading.RLock()

    def __get_frames(self, req):
       return FrameGraph.Response(frame_yaml=self.all_frames_as_yaml())

    def set_transform(self, *args, **kwargs):
        super().set_transform(*args, **kwargs)
        self._call_new_data_callbacks()

    def set_transform_static(self, *args, **kwargs):
        super().set_transform_static(*args, **kwargs)
        self._call_new_data_callbacks()

    def _call_new_data_callbacks(self):
        with self._callbacks_lock:
            for callback in self._new_data_callbacks:
                callback()
        # Remove callbacks after to avoid modifying list being iterated on
            for callback in self._callbacks_to_remove:
                self._new_data_callbacks.remove(callback)

    def _remove_callback(self, callback):
        with self._callbacks_lock:
            # Actually remove the callback later
            self._callbacks_to_remove.append(callback)

    def lookup_transform(self, target_frame, source_frame, time, timeout=Duration()):
        """
        Get the transform from the source frame to the target frame.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """

        self.can_transform(target_frame, source_frame, time, timeout)
        return self.lookup_transform_core(target_frame, source_frame, time)

    async def lookup_transform_async(self, target_frame, source_frame, time):
        """
        Get the transform from the source frame to the target frame asyncronously.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest)
        :return: the transform
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        await self.wait_for_transform_async(target_frame, source_frame, time)
        return self.lookup_transform_core(target_frame, source_frame, time)


    def lookup_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=Duration()):
        """
        Get the transform from the source frame to the target frame using the advanced API.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        self.can_transform_full(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)
        return self.lookup_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)

    async def lookup_transform_full_async(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        """
        Get the transform from the source frame to the target frame using the advanced API asyncronously.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :return: the transform
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        await self.wait_for_transform_full_async(target_frame, target_time, source_frame, source_time, fixed_frame)
        return self.lookup_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)

    def can_transform(self, target_frame, source_frame, time, timeout=Duration(), return_debug_tuple=False):
        """
        Check if a transform from the source frame to the target frame is possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        clock = rclpy.clock.Clock()
        if timeout != Duration():
            start_time = clock.now()
            # TODO(vinnamkim): rclpy.Rate is not ready
            # See https://github.com/ros2/rclpy/issues/186
            # r = rospy.Rate(20)
            while (clock.now() < start_time + timeout and
                   not self.can_transform_core(target_frame, source_frame, time)[0] and
                   (clock.now() + Duration(seconds=3.0)) >= start_time): # big jumps in time are likely bag loops, so break for them
                # r.sleep()
                sleep(0.02)

        core_result = self.can_transform_core(target_frame, source_frame, time)
        if return_debug_tuple:
            return core_result
        return core_result[0]

    def can_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=Duration(),

                           return_debug_tuple=False):
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        clock = rclpy.clock.Clock()
        if timeout != Duration():
            start_time = clock.now()
            # TODO(vinnamkim): rclpy.Rate is not ready
            # See https://github.com/ros2/rclpy/issues/186
            # r = rospy.Rate(20)
            while (clock.now() < start_time + timeout and
                   not self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)[0] and
                   (clock.now() + Duration(seconds=3.0)) >= start_time): # big jumps in time are likely bag loops, so break for them
                # r.sleep()
                sleep(0.02)
        core_result = self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)
        if return_debug_tuple:
            return core_result
        return core_result[0]

    def wait_for_transform_async(self, target_frame, source_frame, time):
        """
        Wait for a transform from the source frame to the target frame to become possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :return: A future that becomes true when the transform is available
        :rtype: rclpy.task.Future
        """
        fut = rclpy.task.Future()
        if self.can_transform_core(target_frame, source_frame, time)[0]:
            # Short cut, the transform is available
            fut.set_result(True)
            return fut

        def _on_new_data():
            try:
                if self.can_transform_core(target_frame, source_frame, time)[0]:
                    fut.set_result(True)
            except BaseException as e:
                fut.set_exception(e)

        self._new_data_callbacks.append(_on_new_data)
        fut.add_done_callback(lambda _: self._remove_callback(_on_new_data))

        return fut

    def wait_for_transform_full_async(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        """
        Wait for a transform from the source frame to the target frame to become possible.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :return: A future that becomes true when the transform is available
        :rtype: rclpy.task.Future
        """
        fut = rclpy.task.Future()
        if self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)[0]:
            # Short cut, the transform is available
            fut.set_result(True)
            return fut

        def _on_new_data():
            try:
                if self.can_transform_full_core(target_frame, target_time, source_frame, source_time, fixed_frame)[0]:
                    fut.set_result(True)
            except BaseException as e:
                fut.set_exception(e)

        self._new_data_callbacks.append(_on_new_data)
        fut.add_done_callback(lambda _: self._remove_callback(_on_new_data))

        return fut
