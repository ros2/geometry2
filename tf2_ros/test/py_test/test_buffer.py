# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import time
import unittest
import rclpy

from rclpy.action import ActionServer
from tf2_ros.buffer import Buffer
from tf2_msgs.action import LookupTransform
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

TIME_FUDGE = 0.3

class TestBufferClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestBuffer', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.feedback = None

    def feedback_callback(self, feedback):
        self.feedback = feedback

    def timed_spin(self, duration):
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

    def execute_goal_callback(self, goal_handle):
        print('execute_goal_callback')
        goal_handle.set_succeeded()
        return LookupTransform.Result()

    def test_can_transform_valid_transform(self):
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        
        from builtin_interfaces.msg import Time

        transform = TransformStamped()
        transform.header.frame_id = "foo"
        transform.header.stamp = rclpy_time.to_msg()
        transform.child_frame_id = "bar"
        transform.transform.translation.x = 42.0
        transform.transform.translation.y = -3.14
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        print(dir(buffer))
        self.assertTrue(buffer.set_transform(transform, "unittest"))
        self.assertTrue(buffer.can_transform("bar", "foo", rclpy_time))
        output = buffer.lookupTransform("foo", "bar", rclpy_time)
        self.assertEqual(transform.child_frame_id, output.child_frame_id)
        self.assertEqual(transform.transform.translation.x, output.transform.translation.x)
        self.assertEqual(transform.transform.translation.y, output.transform.translation.y)
        self.assertEqual(transform.transform.translation.z, output.transform.translation.z)

if __name__ == '__main__':
    unittest.main()
