#!/usr/bin/env python
#
# Copyright 2008 Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import copy
import unittest

import numpy as np
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header
from tf2_ros import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudConversions(unittest.TestCase):

    def setUp(self):
        self.points = np.array([[1, 2, 0], [10, 20, 30]], dtype=np.float32)

        self.point_cloud_in = create_cloud_xyz32(
            Header(frame_id="test"),
            self.points)

    def test_simple_transform(self):
        k = 300.0
        transform_translate_xyz = TransformStamped()
        transform_translate_xyz.transform.translation.x = k
        transform_translate_xyz.transform.translation.y = k
        transform_translate_xyz.transform.translation.z = k
        # no rotation so we only set w
        transform_translate_xyz.transform.rotation.w = 1.0

        self.assertEqual(
            list(read_points(self.point_cloud_in)),
            [(1.0, 2.0, 0.0), (10.0, 20.0, 30.0)])

        # deepcopy is not required here because we have a str for now
        old_data = copy.deepcopy(self.point_cloud_in.data)

        # Apply transform
        point_cloud_transformed = do_transform_cloud(
            self.point_cloud_in, transform_translate_xyz)

        # Expected output
        expected_coordinates = self.points + k

        new_points = list(map(list, read_points(point_cloud_transformed)))
        self.assertEqual(expected_coordinates.tolist(), new_points)
        # checking no modification in input cloud
        self.assertEqual(old_data, self.point_cloud_in.data)

    def test_simple_rotation_transform(self):
        pass


if __name__ == '__main__':
    unittest.main()
