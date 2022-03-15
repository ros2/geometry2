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


import unittest

from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros


class GeometryMsgs(unittest.TestCase):

    def test_transform(self):
        b = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.rotation = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
        t.header.stamp = rclpy.time.Time(seconds=2.0).to_msg()
        t.header.frame_id = 'a'
        t.child_frame_id = 'b'
        b.set_transform(t, 'eitan_rocks')
        out = b.lookup_transform('a',
                                 'b',
                                 rclpy.time.Time(seconds=2.0).to_msg(),
                                 rclpy.time.Duration(seconds=2))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, 'a')
        self.assertEqual(out.child_frame_id, 'b')

        v = PointStamped()
        v.header.stamp = rclpy.time.Time(seconds=2.0).to_msg()
        v.header.frame_id = 'a'
        v.point.x = 1.0
        v.point.y = 2.0
        v.point.z = 3.0
        # b.registration.add(PointStamped)
        out = b.transform(v, 'b', new_type=PointStamped)
        self.assertEqual(out.point.x, 0)
        self.assertEqual(out.point.y, -2)
        self.assertEqual(out.point.z, -3)

        v = PoseStamped()
        v.header.stamp = rclpy.time.Time(seconds=2.0).to_msg()
        v.header.frame_id = 'a'
        v.pose.position.x = 1.0
        v.pose.position.y = 2.0
        v.pose.position.z = 3.0
        v.pose.orientation = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
        out = b.transform(v, 'b')
        self.assertEqual(out.pose.position.x, 0)
        self.assertEqual(out.pose.position.y, -2)
        self.assertEqual(out.pose.position.z, -3)

        v = PoseWithCovarianceStamped()
        v.header.stamp = rclpy.time.Time(seconds=2.0).to_msg()
        v.header.frame_id = 'a'
        v.pose.covariance = (
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0
        )
        v.pose.pose.position.x = 1.0
        v.pose.pose.position.y = 2.0
        v.pose.pose.position.z = 3.0
        v.pose.pose.orientation = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
        out = b.transform(v, 'b')
        self.assertEqual(out.pose.pose.position.x, 0)
        self.assertEqual(out.pose.pose.position.y, -2)
        self.assertEqual(out.pose.pose.position.z, -3)

        # Translation shouldn't affect Vector3
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 3.0
        t.transform.rotation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        v = Vector3Stamped()
        v.vector.x = 1.0
        v.vector.y = 0.0
        v.vector.z = 0.0
        out = tf2_geometry_msgs.do_transform_vector3(v, t)
        self.assertEqual(out.vector.x, 1)
        self.assertEqual(out.vector.y, 0)
        self.assertEqual(out.vector.z, 0)

        # Rotate Vector3 180 deg about y
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 3.0
        t.transform.rotation = Quaternion(w=0.0, x=0.0, y=1.0, z=0.0)

        v = Vector3Stamped()
        v.vector.x = 1.0
        v.vector.y = 0.0
        v.vector.z = 0.0

        out = tf2_geometry_msgs.do_transform_vector3(v, t)
        self.assertEqual(out.vector.x, -1)
        self.assertEqual(out.vector.y, 0)
        self.assertEqual(out.vector.z, 0)

        # Testing for pose and covariance transform
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 3.0
        t.transform.rotation = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)

        v = PoseWithCovarianceStamped()
        v.header.stamp = rclpy.time.Time(seconds=2.0).to_msg()
        v.header.frame_id = 'a'
        v.pose.covariance = (
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0
        )
        v.pose.pose.position.x = 1.0
        v.pose.pose.position.y = 2.0
        v.pose.pose.position.z = 3.0
        v.pose.pose.orientation = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)

        out = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(v, t)
        expected_covariance = np.array([
          1.0, -2.0, -3.0, 4.0, -5.0, -6.0,
          -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
          -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
          1.0, -2.0, -3.0, 4.0, -5.0, -6.0,
          -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
          -1.0, 2.0, 3.0, -4.0, 5.0, 6.0
        ])
        self.assertEqual(out.pose.pose.position.x, 2)
        self.assertEqual(out.pose.pose.position.y, 0)
        self.assertEqual(out.pose.pose.position.z, 0)
        self.assertEqual(out.pose.pose.orientation.x, 0)
        self.assertEqual(out.pose.pose.orientation.y, 0)
        self.assertEqual(out.pose.pose.orientation.z, 0)
        self.assertEqual(out.pose.pose.orientation.w, 1)
        self.assertTrue(np.array_equal(out.pose.covariance, expected_covariance))


if __name__ == '__main__':
    rclpy.init(args=None)
