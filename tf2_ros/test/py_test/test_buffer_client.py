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

import time
import unittest
import rclpy
import threading

from tf2_ros.buffer_client import BufferClient
from geometry_msgs.msg import TransformStamped
from tf2_msgs.action import LookupTransform
from tf2_py import BufferCore, TransformException, TimeoutException, \
    LookupException, InvalidArgumentException, ExtrapolationException, ConnectivityException
from rclpy.executors import SingleThreadedExecutor
from tf2_msgs.msg import TF2Error


def build_transform(target_frame, source_frame, stamp):
    transform = TransformStamped()
    transform.header.frame_id = target_frame
    transform.header.stamp = stamp
    transform.child_frame_id = source_frame

    transform.transform.translation.x = 42.0
    transform.transform.translation.y = -3.14
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0

    return transform


class MockActionServer():
    def __init__(self, node, buffer_core):
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
        self.node = node
        self.buffer_core = buffer_core
        self.result_buffer = {}

    def goal_callback(self, request, response):
        response.accepted = True
        bytes_goal_id = bytes(request.goal_id.uuid)
        try:
            if not request.goal.advanced:
                transform = self.buffer_core.lookup_transform_core(target_frame=request.goal.target_frame,
                                                                   source_frame=request.goal.source_frame,
                                                                   time=request.goal.source_time)
                self.result_buffer[bytes_goal_id] = (
                    transform, TF2Error.NO_ERROR, '')
            else:
                transform = self.buffer_core.lookup_transform_full_core(
                    target_frame=request.goal.target_frame,
                    source_frame=request.goal.source_frame,
                    source_time=request.goal.source_time,
                    target_time=request.goal.target_time,
                    fixed_frame=request.goal.fixed_frame
                )
                self.result_buffer[bytes_goal_id] = (
                    transform, TF2Error.NO_ERROR, ''
                )
        except TimeoutException as e:
            self.result_buffer[bytes_goal_id] = (
                TransformStamped(), TF2Error.TIMEOUT_ERROR, e)
        except LookupException as e:
            self.result_buffer[bytes_goal_id] = (
                TransformStamped(), TF2Error.LOOKUP_ERROR, e)
        except InvalidArgumentException as e:
            self.result_buffer[bytes_goal_id] = (
                TransformStamped(), TF2Error.INVALID_ARGUMENT_ERROR, e)
        except ExtrapolationException as e:
            self.result_buffer[bytes_goal_id] = (
                TransformStamped(), TF2Error.EXTRAPOLATION_ERROR, e)
        except ConnectivityException as e:
            self.result_buffer[bytes_goal_id] = (
                TransformStamped(), TF2Error.CONNECTIVITY_ERROR, e)
        except TransformException as e:
            self.result_buffer[bytes_goal_id] = (
                TransformStamped(), TF2Error.TRANSFORM_ERROR, e)

        return response

    def cancel_callback(self, request, response):
        response.goals_canceling.append(request.goal_info)
        return response

    def result_callback(self, request, response):
        bytes_goal_id = bytes(request.goal_id.uuid)
        #response.status = TF2Error
        response.result.transform = self.result_buffer[bytes_goal_id][0]
        response.result.error = TF2Error(
            error=self.result_buffer[bytes_goal_id][1],
            error_string=str(self.result_buffer[bytes_goal_id][2]))
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
        cls.executor.add_node(cls.node)

        buffer_core = BufferCore()
        transform = build_transform('foo', 'bar', rclpy.time.Time().to_msg())
        buffer_core.set_transform(transform, 'unittest')

        cls.mock_action_server = MockActionServer(cls.node, buffer_core)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.spinning = threading.Event()
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()
        return

    def tearDown(self):
        self.spinning.set()
        self.spin_thread.join()
        #print('teardown : ', self.spin_thread.is_alive(), self.context.ok())
        return

    def feedback_callback(self, feedback):
        self.feedback = feedback

    def spin(self):
        try:
            while self.context.ok() and not self.spinning.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            return

    def timed_spin(self, duration):
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

    def execute_goal_callback(self, goal_handle):
        print('execute_goal_callback')
        goal_handle.set_succeeded()
        return LookupTransform.Result()

    def test_lookup_transform_true(self):
        buffer_client = BufferClient(
            self.node, 'lookup_transform', check_frequency=10.0, timeout_padding=0.0)

        result = buffer_client.lookup_transform(
            'foo', 'bar', rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))

        self.assertEqual(build_transform(
            'foo', 'bar', rclpy.time.Time().to_msg()), result)

    def test_lookup_transform_fail(self):
        buffer_client = BufferClient(
            self.node, 'lookup_transform', check_frequency=10.0, timeout_padding=0.0)

        with self.assertRaises(LookupException) as ex:
            result = buffer_client.lookup_transform(
                'bar', 'baz', rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))

        self.assertEqual(LookupException, type(ex.exception))

if __name__ == '__main__':
    unittest.main()
