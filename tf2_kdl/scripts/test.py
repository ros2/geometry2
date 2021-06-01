#!/usr/bin/python

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

import builtin_interfaces
from geometry_msgs.msg import Quaternion, TransformStamped
import PyKDL
import rclpy
import tf2_ros
import tf2_kdl  # noqa(F401)


class KDLConversions(unittest.TestCase):

    def test_transform(self):
        b = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.rotation = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
        t.header.stamp = builtin_interfaces.msg.Time(sec=2)
        t.header.frame_id = 'a'
        t.child_frame_id = 'b'
        b.set_transform(t, 'eitan_rocks')
        out = b.lookup_transform('a', 'b',
                                 rclpy.time.Time(seconds=2),
                                 rclpy.time.Duration(seconds=2))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, 'a')
        self.assertEqual(out.child_frame_id, 'b')

        v = PyKDL.Vector(1, 2, 3)
        out = b.transform(
            tf2_ros.Stamped(v, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.x(), 0)
        self.assertEqual(out.y(), -2)
        self.assertEqual(out.z(), -3)

        f = PyKDL.Frame(PyKDL.Rotation.RPY(1, 2, 3), PyKDL.Vector(1, 2, 3))
        out = b.transform(
            tf2_ros.Stamped(f, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.p.x(), 0)
        self.assertEqual(out.p.y(), -2)
        self.assertEqual(out.p.z(), -3)
        # TODO(tfoote) check values of rotation

        t = PyKDL.Twist(PyKDL.Vector(1, 2, 3), PyKDL.Vector(4, 5, 6))
        out = b.transform(
            tf2_ros.Stamped(t, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.vel.x(), 1)
        self.assertEqual(out.vel.y(), -8)
        self.assertEqual(out.vel.z(), 2)
        self.assertEqual(out.rot.x(), 4)
        self.assertEqual(out.rot.y(), -5)
        self.assertEqual(out.rot.z(), -6)

        w = PyKDL.Wrench(PyKDL.Vector(1, 2, 3), PyKDL.Vector(4, 5, 6))
        out = b.transform(
            tf2_ros.Stamped(w, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.force.x(), 1)
        self.assertEqual(out.force.y(), -2)
        self.assertEqual(out.force.z(), -3)
        self.assertEqual(out.torque.x(), 4)
        self.assertEqual(out.torque.y(), -8)
        self.assertEqual(out.torque.z(), -4)

    def test_convert(self):
        v = PyKDL.Vector(1, 2, 3)
        vs = tf2_ros.Stamped(v, builtin_interfaces.msg.Time(sec=2), 'a')
        vs2 = tf2_ros.convert(vs, PyKDL.Vector)
        self.assertEqual(vs.x(), 1)
        self.assertEqual(vs.y(), 2)
        self.assertEqual(vs.z(), 3)
        self.assertEqual(vs2.x(), 1)
        self.assertEqual(vs2.y(), 2)
        self.assertEqual(vs2.z(), 3)


if __name__ == '__main__':
    rclpy.init(args=None)
    node = rclpy.create_node('test_tf2_kdl_python')
    unittest.main()
