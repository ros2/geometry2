#!/usr/bin/python3
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
import sys
import unittest

import tf2_py as tf2
import tf2_ros
from geometry_msgs.msg import PointStamped
import rclpy
import tf2_geometry_msgs
# TODO (ahcorde): Enable once python_orocos_kdl is ported
# import tf2_kdl
# import PyKDL
from rclpy.executors import SingleThreadedExecutor
import threading

class TestBufferClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestBufferClient', context=cls.context)
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.spinning = threading.Event()
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()
        return

    def tearDown(self):
        self.spinning.set()
        self.spin_thread.join()
        return

    def spin(self):
        try:
            while self.context.ok() and not self.spinning.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            return

    def test_buffer_client(self):
        buffer_client = tf2_ros.BufferClient(
            self.node, '/tf_action', check_frequency=10.0, timeout_padding=0.0)

        p1 = PointStamped()
        p1.header.frame_id = "a"
        p1.header.stamp = rclpy.time.Time(seconds=1.0).to_msg()
        p1.point.x = 0.0
        p1.point.y = 0.0
        p1.point.z = 0.0

        buffer_client.action_client.wait_for_server()

        try:
            p2 = buffer_client.transform(p1, "b", timeout=rclpy.time.Duration(seconds=1))
            self.node.get_logger().info("p1: %s, p2: %s" % (p1, p2))
        except tf2.TransformException as e:
            self.node.get_logger().error("%s" % e)
            self.assertEqual(0, 4)

    # TODO (ahcorde): Enable once python_orocos_kdl is ported
    # def test_transform_type(self):
    #     buffer_client = tf2_ros.BufferClient(
    #         self.node, '/tf_action', check_frequency=10.0, timeout_padding=0.0)
    #
    #     p1 = PointStamped()
    #     p1.header.frame_id = "a"
    #     p1.header.stamp = rclpy.time.Time(seconds=1.0).to_msg()
    #     p1.point.x = 0.0
    #     p1.point.y = 0.0
    #     p1.point.z = 0.0
    #
    #     buffer_client.action_client.wait_for_server()
    #
    #     try:
    #         p2 = buffer_client.transform(p1, "b", timeout=rclpy.time.Duration(seconds=1),
    #                                      new_type = PyKDL.Vector)
    #         self.node.get_logger().info("p1: %s, p2: %s" % (str(p1), str(p2)))
    #     except tf2.TransformException as e:
    #         self.node.get_logger().error("%s" % e)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    sys.argv = [sys.argv[0]]
    unittest.main()
