# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
from typing import Optional
from typing import Union
from typing import List

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class StaticTransformBroadcaster:
    """
    :class:`StaticTransformBroadcaster` is a convenient way to send static transformation on the ``"/tf_static"`` message topic.
    """

    def __init__(self, node: Node, qos: Optional[Union[QoSProfile, int]] = None) -> None:
        """
        Constructor.

        :param node: The ROS2 node.
        :param qos: A QoSProfile or a history depth to apply to the publisher.
        """
        if qos is None:
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                )
        self.pub_tf = node.create_publisher(TFMessage, "/tf_static", qos)

        self.net_message = TFMessage()
        self._child_frame_ids = set()

    def sendTransform(self, transform: Union[TransformStamped, List[TransformStamped]]) -> None:
        if not isinstance(transform, list):
            if hasattr(transform, '__iter__'):
                transform = list(transform)
            else:
                transform = [transform]

        for t_in in transform:
            if t_in.child_frame_id not in self._child_frame_ids:
                self._child_frame_ids.add(t_in.child_frame_id)
                self.net_message.transforms.append(t_in)

        self.pub_tf.publish(self.net_message)
