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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `base_link` to a platform and sensor
    frames.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('example_static_frame_publisher')

        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self._tf_publisher.sendTransform(self.make_transforms())

    def make_transforms(self):
        base_link_to_platform = TransformStamped()
        base_link_to_platform.header.frame_id = 'base_link'
        base_link_to_platform.header.stamp = self.get_clock().now().to_msg()
        base_link_to_platform.child_frame_id = 'platform'
        base_link_to_platform.transform.translation.x = 0.0
        base_link_to_platform.transform.translation.y = 0.0
        base_link_to_platform.transform.translation.z = 0.1
        base_link_to_platform.transform.rotation.w = 1.0
        base_link_to_platform.transform.rotation.x = 0.0
        base_link_to_platform.transform.rotation.y = 0.0
        base_link_to_platform.transform.rotation.z = 0.0

        platform_to_camera = TransformStamped()
        platform_to_camera.header.frame_id = 'platform'
        platform_to_camera.header.stamp = self.get_clock().now().to_msg()
        platform_to_camera.child_frame_id = 'camera'
        platform_to_camera.transform.translation.x = 0.05
        platform_to_camera.transform.translation.y = 0.0
        platform_to_camera.transform.translation.z = 0.01
        platform_to_camera.transform.rotation.w = 1.0
        platform_to_camera.transform.rotation.x = 0.0
        platform_to_camera.transform.rotation.y = 0.0
        platform_to_camera.transform.rotation.z = 0.0

        platform_to_sonar_0 = TransformStamped()
        platform_to_sonar_0.header.frame_id = 'platform'
        platform_to_sonar_0.header.stamp = self.get_clock().now().to_msg()
        platform_to_sonar_0.child_frame_id = 'sonar_0'
        platform_to_sonar_0.transform.translation.x = 0.05 * math.sin(math.radians(45))
        platform_to_sonar_0.transform.translation.y = 0.05 * math.cos(math.radians(45))
        platform_to_sonar_0.transform.translation.z = 0.0
        platform_to_sonar_0.transform.rotation.w = -0.8733046400935156
        platform_to_sonar_0.transform.rotation.x = 0.0
        platform_to_sonar_0.transform.rotation.y = 0.0
        platform_to_sonar_0.transform.rotation.z = -0.4871745124605095

        platform_to_sonar_1 = TransformStamped()
        platform_to_sonar_1.header.frame_id = 'platform'
        platform_to_sonar_1.header.stamp = self.get_clock().now().to_msg()
        platform_to_sonar_1.child_frame_id = 'sonar_1'
        platform_to_sonar_1.transform.translation.x = 0.05
        platform_to_sonar_1.transform.translation.y = 0.0
        platform_to_sonar_1.transform.translation.z = 0.0
        platform_to_sonar_1.transform.rotation.w = 1.0
        platform_to_sonar_1.transform.rotation.x = 0.0
        platform_to_sonar_1.transform.rotation.y = 0.0
        platform_to_sonar_1.transform.rotation.z = 0.0

        platform_to_sonar_2 = TransformStamped()
        platform_to_sonar_2.header.frame_id = 'platform'
        platform_to_sonar_2.header.stamp = self.get_clock().now().to_msg()
        platform_to_sonar_2.child_frame_id = 'sonar_2'
        platform_to_sonar_2.transform.translation.x = 0.05 * math.sin(math.radians(45))
        platform_to_sonar_2.transform.translation.y = -0.05 * math.cos(math.radians(45))
        platform_to_sonar_2.transform.translation.z = 0.0
        platform_to_sonar_2.transform.rotation.w = 0.8733046400935156
        platform_to_sonar_2.transform.rotation.x = 0.0
        platform_to_sonar_2.transform.rotation.y = 0.0
        platform_to_sonar_2.transform.rotation.z = -0.4871745124605095

        return (
            base_link_to_platform,
            platform_to_camera,
            platform_to_sonar_0,
            platform_to_sonar_1,
            platform_to_sonar_2,
        )


def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
