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
from tf2_ros.buffer_client import BufferClient
from tf2_msgs.action import LookupTransform

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

TIME_FUDGE = 0.3

class MockActionServer():
    def __init__(self, node):
        self.goal_srv = node.create_service(
            LookupTransform.Impl.SendGoalService, '/lookup_transform/_action/send_goal',
            self.goal_callback)
        self.cancel_srv = node.create_service(
            LookupTransform.Impl.CancelGoalService, '/lookup_transform/_action/cancel_goal',
            self.cancel_callback)
        self.result_srv = node.create_service(
            LookupTransform.Impl.GetResultService, '/lookup_transform/_action/get_result',
            self.result_callback)
        self.feedback_pub = node.create_publisher(
            LookupTransform.Impl.FeedbackMessage, '/lookup_transform/_action/feedback')

    def goal_callback(self, request, response):
        response.accepted = True
        return response

    def cancel_callback(self, request, response):
        response.goals_canceling.append(request.goal_info)
        return response

    def result_callback(self, request, response):
        return response

    def publish_feedback(self, goal_id):
        feedback_message = LookupTransform.Impl.FeedbackMessage()
        feedback_message.goal_id = goal_id
        self.feedback_pub.publish(feedback_message)

class TestBufferClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestBufferClient', context=cls.context)
        cls.mock_action_server = MockActionServer(cls.node)

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

    def test_wait_for_server_5sec(self):
        buffer_client = BufferClient(
            self.node, 'lookup_transform', check_frequency=10.0, timeout_padding=0.0)
        
        try:
            start = time.monotonic()
            self.assertFalse(buffer_client.wait_for_server(5.0))
            end = time.monotonic()
            self.assertGreater(5.0, end - start - TIME_FUDGE)
            self.assertLess(5.0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(buffer_client)

    def test_wait_for_service_nowait(self):
        cli = self.node.create_client(GetParameters, 'get/parameters')
        try:
            start = time.monotonic()
            self.assertFalse(cli.wait_for_service(timeout_sec=0))
            end = time.monotonic()
            self.assertGreater(0, end - start - TIME_FUDGE)
            self.assertLess(0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(cli)

    def test_wait_for_service_exists(self):
        cli = self.node.create_client(GetParameters, 'test_wfs_exists')
        srv = self.node.create_service(
            GetParameters, 'test_wfs_exists', lambda request: None)
        try:
            start = time.monotonic()
            self.assertTrue(cli.wait_for_service(timeout_sec=1.0))
            end = time.monotonic()
            self.assertGreater(0, end - start - TIME_FUDGE)
            self.assertLess(0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)

    def test_concurrent_calls_to_service(self):
        cli = self.node.create_client(GetParameters, 'get/parameters')
        srv = self.node.create_service(
            GetParameters, 'get/parameters',
            lambda request, response: response)
        try:
            self.assertTrue(cli.wait_for_service(timeout_sec=20))
            future1 = cli.call_async(GetParameters.Request())
            future2 = cli.call_async(GetParameters.Request())
            executor = rclpy.executors.SingleThreadedExecutor(
                context=self.context)
            rclpy.spin_until_future_complete(
                self.node, future1, executor=executor)
            rclpy.spin_until_future_complete(
                self.node, future2, executor=executor)
            self.assertTrue(future1.result() is not None)
            self.assertTrue(future2.result() is not None)
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)


if __name__ == '__main__':
    unittest.main()
