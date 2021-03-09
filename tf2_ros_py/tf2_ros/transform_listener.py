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
from typing import Optional
from typing import Union

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_msgs.msg import TFMessage
from threading import Thread


class TransformListener:
    """
    :class:`TransformListener` is a convenient way to listen for coordinate frame transformation info.
    This class takes an object that instantiates the :class:`BufferInterface` interface, to which
    it propagates changes to the tf frame graph.
    """
    def __init__(
        self,
        buffer: Buffer,
        node: Node,
        *,
        spin_thread: bool = False,
        qos: Optional[Union[QoSProfile, int]] = None,
        static_qos: Optional[Union[QoSProfile, int]] = None
    ) -> None:
        """
        Constructor.

        :param buffer: The buffer to propagate changes to when tf info updates.
        :param node: The ROS2 node.
        :param spin_thread: Whether to create a dedidcated thread to spin this node.
        :param qos: A QoSProfile or a history depth to apply to subscribers.
        :param static_qos: A QoSProfile or a history depth to apply to tf_static subscribers.
        """
        if qos is None:
            qos = QoSProfile(
                depth=100,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                )
        if static_qos is None:
            static_qos = QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                )
        self.buffer = buffer
        self.node = node
        # Default callback group is mutually exclusive, which would prevent waiting for transforms
        # from another callback in the same group.
        self.group = ReentrantCallbackGroup()
        self.tf_sub = node.create_subscription(
            TFMessage, '/tf', self.callback, qos, callback_group=self.group)
        self.tf_static_sub = node.create_subscription(
            TFMessage, '/tf_static', self.static_callback, static_qos, callback_group=self.group)

        if spin_thread:
            self.executor = SingleThreadedExecutor()

            def run_func():
                self.executor.add_node(self.node)
                self.executor.spin()
                self.executor.remove_node(self.node)

            self.dedicated_listener_thread = Thread(target=run_func)
            self.dedicated_listener_thread.start()

    def __del__(self) -> None:
        if hasattr(self, 'dedicated_listener_thread') and hasattr(self, 'executor'):
            self.executor.shutdown()
            self.dedicated_listener_thread.join()

        self.unregister()

    def unregister(self) -> None:
        """
        Unregisters all tf subscribers.
        """
        self.node.destroy_subscription(self.tf_sub)
        self.node.destroy_subscription(self.tf_static_sub)

    def callback(self, data: TFMessage) -> None:
        who = 'default_authority'
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data: TFMessage) -> None:
        who = 'default_authority'
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)
