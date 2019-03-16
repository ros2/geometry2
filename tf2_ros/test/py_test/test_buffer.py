# ***********************************************************
# * Software License Agreement (BSD License)
# *
# *  Copyright (c) 2009, Willow Garage, Inc.
# *  All rights reserved.
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions
# *  are met:
# *
# *   * Redistributions of source code must retain the above copyright
# *     notice, this list of conditions and the following disclaimer.
# *   * Redistributions in binary form must reproduce the above
# *     copyright notice, this list of conditions and the following
# *     disclaimer in the documentation and/or other materials provided
# *     with the distribution.
# *   * Neither the name of Willow Garage, Inc. nor the names of its
# *     contributors may be used to endorse or promote products derived
# *     from this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# *  POSSIBILITY OF SUCH DAMAGE.
# *
# * Author: Vinnam Kim vinnam.kim@gmail.com
# ***********************************************************

import unittest
import rclpy

from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped, PointStamped

class TestBuffer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_can_transform_valid_transform(self):
        buffer = Buffer()
        clock = rclpy.clock.Clock()
        rclpy_time = clock.now()
        
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
        
        self.assertEqual(buffer.set_transform(transform, "unittest"), None)

        self.assertEqual(buffer.can_transform("foo", "bar", rclpy_time), True)
        
        output = buffer.lookup_transform("foo", "bar", rclpy_time)
        self.assertEqual(transform.child_frame_id, output.child_frame_id)
        self.assertEqual(transform.transform.translation.x, output.transform.translation.x)
        self.assertEqual(transform.transform.translation.y, output.transform.translation.y)
        self.assertEqual(transform.transform.translation.z, output.transform.translation.z)

if __name__ == '__main__':
    unittest.main()
