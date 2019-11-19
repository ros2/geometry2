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

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros.transform_broadcaster import TransformBroadcaster


class DynamicFramePublisher(Node):
    """
    Broadcast transforms that change regularly.

    This example publishes transforms from `base_link` to two wheel frames:
    port_wheel, and starboard wheel.
    The transform is computed from received joint state messages.
    """

    def __init__(self):
        super().__init__('example_dynamic_frame_publisher')

        self._tf_publisher = TransformBroadcaster(self)

        qos = QoSProfile(depth=10)
        self._joint_state_sub = self.create_subscription(
            JointState, 'joints', self.on_joint_state, qos)

    def on_joint_state(self, joint_state_msg):
        transforms = []

        for joint, position in zip(joint_state_msg.name, joint_state_msg.position):
            if 'port_wheel' == joint or 'starboard_wheel' == joint:
                tfs = TransformStamped()
                tfs.header.frame_id = 'base_link'
                tfs.header.stamp = joint_state_msg.header.stamp
                tfs.child_frame_id = joint
                tfs.transform.translation.x = 0.0
                if 'port_wheel' == joint:
                    tfs.transform.translation.y = 0.05
                else:
                    tfs.transform.translation.y = -0.05
                tfs.transform.translation.z = 0.0
                tfs.transform.rotation.w = math.cos(position / 2)
                tfs.transform.rotation.x = 0.0
                tfs.transform.rotation.y = math.sin(position / 2)
                tfs.transform.rotation.z = 0.0
                transforms.append(tfs)

        if transforms:
            self._tf_publisher.sendTransform(transforms)


class FakeJointStatePublisher(Node):
    """Publish fake joint states to make the example publish transforms."""

    def __init__(self):
        super().__init__('fake_joint_state_publisher')

        qos = QoSProfile(depth=1)
        self._joint_state_pub = self.create_publisher(JointState, 'joints', qos)

        # Publish regularly for smooth looking frames in rviz
        self._tf_tmr = self.create_timer(1 / 30.0, self.on_timer)

    def on_timer(self):
        msg = JointState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.name = ['port_wheel', 'starboard_wheel']
        msg.position = [now.nanoseconds / 1e9, now.nanoseconds / 2e9 + 0.5]
        self._joint_state_pub.publish(msg)


def main():
    rclpy.init()
    nodes = []
    nodes.append(DynamicFramePublisher())
    nodes.append(FakeJointStatePublisher())

    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
