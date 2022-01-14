#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*
#* Author: Eitan Marder-Eppstein
#***********************************************************
from typing import TypeVar

from geometry_msgs.msg import TransformStamped

from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.clock import Clock
from time import sleep

import builtin_interfaces.msg
import tf2_py as tf2
import tf2_ros
import threading
import warnings

from tf2_msgs.action import LookupTransform

# Used for documentation purposes only
LookupTransformGoal = TypeVar('LookupTransformGoal')
LookupTransformResult = TypeVar('LookupTransformResult')


class BufferClient(tf2_ros.BufferInterface):
    """
    Action client-based implementation of BufferInterface.
    """
    def __init__(
        self,
        node: Node,
        ns: str,
        check_frequency: float = 10.0,
        timeout_padding: Duration = Duration(seconds=2.0)
    ) -> None:
        """
        Constructor.

        :param node: The ROS2 node.
        :param ns: The namespace in which to look for a BufferServer.
        :param check_frequency: How frequently to check for updates to known transforms.
        :param timeout_padding: A constant timeout to add to blocking calls.
        """
        tf2_ros.BufferInterface.__init__(self)
        self.node = node
        self.action_client = ActionClient(node, LookupTransform, action_name=ns)
        self.check_frequency = check_frequency
        self.timeout_padding = timeout_padding

    # lookup, simple api
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
        if isinstance(time, builtin_interfaces.msg.Time):
            source_time = Time.from_msg(time)
            warnings.warn(
                'Passing a builtin_interfaces.msg.Time argument is deprecated, and will be removed in the near future. '
                'Use rclpy.time.Time instead.')
        elif isinstance(time, Time):
            source_time = time
        else:
            raise TypeError('Must pass a rclpy.time.Time object.')

        goal = LookupTransform.Goal()
        goal.target_frame = target_frame
        goal.source_frame = source_frame
        goal.source_time = source_time.to_msg()
        goal.timeout = timeout.to_msg()
        goal.advanced = False

        return self.__process_goal(goal)

    # lookup, advanced api
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
        :param target_time: The time to transform to. (0 will get the latest)
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest)
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: Time to wait for the target frame to become available.
        :return: The transform between the frames.
        """
        goal = LookupTransform.Goal()
        goal.target_frame = target_frame
        goal.source_frame = source_frame
        goal.source_time = source_time.to_msg()
        goal.timeout = timeout.to_msg()
        goal.target_time = target_time.to_msg()
        goal.fixed_frame = fixed_frame
        goal.advanced = True

        return self.__process_goal(goal)

    # can, simple api
    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Duration = Duration()
    ) -> bool:
        """
        Check if a transform from the source frame to the target frame is possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest)
        :param timeout: Time to wait for the target frame to become available.
        :param return_debug_type: If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        """
        try:
            self.lookup_transform(target_frame, source_frame, time, timeout)
            return True
        except tf2.TransformException:
            return False

    # can, advanced api
    def can_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Duration = Duration()
    ) -> bool:
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest)
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest)
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: Time to wait for the target frame to become available.
        :param return_debug_type: If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        """
        try:
            self.lookup_transform_full(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)
            return True
        except tf2.TransformException:
            return False

    def __process_goal(self, goal: LookupTransformGoal) -> TransformStamped:
        # TODO(sloretz) why is this an action client? Service seems more appropriate.
        if not self.action_client.server_is_ready():
            raise tf2.TimeoutException("The BufferServer is not ready.")

        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(unblock)

        def unblock_by_timeout():
            nonlocal send_goal_future, goal, event
            clock = Clock()
            start_time = clock.now()
            timeout = Duration.from_msg(goal.timeout)
            timeout_padding = self.timeout_padding
            while not send_goal_future.done() and not event.is_set():
                if clock.now() > start_time + timeout + timeout_padding:
                    break
                # TODO(Anyone): We can't use Rate here because it would never expire
                # with a single-threaded executor.
                # See https://github.com/ros2/geometry2/issues/327 for ideas on
                # how to timeout waiting for transforms that don't block the executor.
                sleep(1.0 / self.check_frequency)

            event.set()

        t = threading.Thread(target=unblock_by_timeout)
        t.start()

        event.wait()

        # This shouldn't happen, but could in rare cases where the server hangs
        if not send_goal_future.done():
            raise tf2.TimeoutException("The LookupTransform goal sent to the BufferServer did not come back in the specified time. Something is likely wrong with the server.")

        # Raises if future was given an exception
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            raise tf2.TimeoutException("The LookupTransform goal sent to the BufferServer did not come back with accepted status. Something is likely wrong with the server.")

        response = self.action_client._get_result(goal_handle)

        return self.__process_result(response.result)

    def __process_result(self, result: LookupTransformResult) -> TransformStamped:
        if result == None or result.error == None:
            raise tf2.TransformException("The BufferServer returned None for result or result.error!  Something is likely wrong with the server.")
        if result.error.error != result.error.NO_ERROR:
            if result.error.error == result.error.LOOKUP_ERROR:
                raise tf2.LookupException(result.error.error_string)
            if result.error.error == result.error.CONNECTIVITY_ERROR:
                raise tf2.ConnectivityException(result.error.error_string)
            if result.error.error == result.error.EXTRAPOLATION_ERROR:
                raise tf2.ExtrapolationException(result.error.error_string)
            if result.error.error == result.error.INVALID_ARGUMENT_ERROR:
                raise tf2.InvalidArgumentException(result.error.error_string)
            if result.error.error == result.error.TIMEOUT_ERROR:
                raise tf2.TimeoutException(result.error.error_string)

            raise tf2.TransformException(result.error.error_string)

        return result.transform

    def destroy(self) -> None:
        """Cleanup resources associated with this BufferClient."""

        self.action_client.destroy()
