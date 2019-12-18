#!/usr/bin/python

import unittest
import rclpy
import PyKDL
import tf2_ros
import tf2_kdl
from geometry_msgs.msg import TransformStamped, Quaternion
from copy import deepcopy
import builtin_interfaces

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
        out = b.lookup_transform('a','b', rclpy.time.Time(seconds=2),  rclpy.time.Duration(seconds=2))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, 'a')
        self.assertEqual(out.child_frame_id, 'b')

        v = PyKDL.Vector(1,2,3)
        out = b.transform(tf2_ros.Stamped(v, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.x(), 0)
        self.assertEqual(out.y(), -2)
        self.assertEqual(out.z(), -3)

        f = PyKDL.Frame(PyKDL.Rotation.RPY(1,2,3), PyKDL.Vector(1,2,3))
        out = b.transform(tf2_ros.Stamped(f, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.p.x(), 0)
        self.assertEqual(out.p.y(), -2)
        self.assertEqual(out.p.z(), -3)
        # TODO(tfoote) check values of rotation

        t = PyKDL.Twist(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
        out = b.transform(tf2_ros.Stamped(t, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.vel.x(), 1)
        self.assertEqual(out.vel.y(), -8)
        self.assertEqual(out.vel.z(), 2)
        self.assertEqual(out.rot.x(), 4)
        self.assertEqual(out.rot.y(), -5)
        self.assertEqual(out.rot.z(), -6)

        w = PyKDL.Wrench(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
        out = b.transform(tf2_ros.Stamped(w, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        self.assertEqual(out.force.x(), 1)
        self.assertEqual(out.force.y(), -2)
        self.assertEqual(out.force.z(), -3)
        self.assertEqual(out.torque.x(), 4)
        self.assertEqual(out.torque.y(), -8)
        self.assertEqual(out.torque.z(), -4)

    def test_convert(self):
        v = PyKDL.Vector(1,2,3)
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
