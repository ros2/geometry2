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
import struct
import unittest

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from tf2_ros import TransformStamped
import tf2_sensor_msgs


# A sample python unit test
class PointCloudConversions(unittest.TestCase):

    def setUp(self):
        self.point_cloud_in = point_cloud2.PointCloud2()
        self.point_cloud_in.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]

        self.point_cloud_in.point_step = 4 * 3
        self.point_cloud_in.height = 1
        # we add two points (with x, y, z to the cloud)
        self.point_cloud_in.width = 2
        self.point_cloud_in.row_step = \
            self.point_cloud_in.point_step * self.point_cloud_in.width

        points = [1, 2, 0, 10, 20, 30]
        self.point_cloud_in.data = struct.pack('%sf' % len(points), *points)

        self.transform_translate_xyz_300 = TransformStamped()
        self.transform_translate_xyz_300.transform.translation.x = 300
        self.transform_translate_xyz_300.transform.translation.y = 300
        self.transform_translate_xyz_300.transform.translation.z = 300
        # no rotation so we only set w
        self.transform_translate_xyz_300.transform.rotation.w = 1

        assert(list(point_cloud2.read_points(self.point_cloud_in)) ==
               [(1.0, 2.0, 0.0), (10.0, 20.0, 30.0)])

    def test_simple_transform(self):
        # deepcopy is not required here because we have a str for now
        old_data = copy.deepcopy(self.point_cloud_in.data)
        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(
            self.point_cloud_in, self.transform_translate_xyz_300)

        k = 300
        expected_coordinates = [(1+k, 2+k, 0+k), (10+k, 20+k, 30+k)]
        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        print('new_points are %s' % new_points)
        assert(expected_coordinates == new_points)
        # checking no modification in input cloud
        assert(old_data == self.point_cloud_in.data)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('test_tf2_sensor_msgs', 'test_point_cloud_conversion',
                    PointCloudConversions)
