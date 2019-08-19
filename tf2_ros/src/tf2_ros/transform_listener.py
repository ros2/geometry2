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

from rclpy.node import Node
import tf2_ros
from tf2_msgs.msg import TFMessage

class TransformListener(Node):
    """
    :class:`TransformListener` is a convenient way to listen for coordinate frame transformation info.
    This class takes an object that instantiates the :class:`BufferInterface` interface, to which
    it propagates changes to the tf frame graph.
    """
    def __init__(self, buffer, name=None):
        """
        .. function:: __init__(buffer, name=None)

            Constructor.

            :param buffer: The buffer to propagate changes to when tf info updates.
            :param name: The name of ROS2 node.
        """
        super().__init__('transform_listener_impl' if name is None else name)
        self.buffer = buffer
        self.tf_sub = self.create_subscription(TFMessage, 'tf', self.callback)
        self.tf_static_sub = self.create_subscription(TFMessage, 'tf_static', self.static_callback)
    
    def __del__(self):
        self.unregister()

    def unregister(self):
        """
        Unregisters all tf subscribers.
        """
        self.destroy_subscription(self.tf_sub)
        self.destroy_subscription(self.tf_static_sub)

    def callback(self, data):
        who = str("default_authority")
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data):
        who = str("default_authority")
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)
