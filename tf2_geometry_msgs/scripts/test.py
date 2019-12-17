#!/usr/bin/env python3

import unittest
import rclpy
import PyKDL
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped, Vector3Stamped, PoseStamped, Quaternion

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
        out = b.lookup_transform('a','b', rclpy.time.Time(seconds=2.0).to_msg(), rclpy.time.Duration(seconds=2))
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
        out = b.transform(v, 'b', new_type = PointStamped)
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

if __name__ == '__main__':
    rclpy.init(args=None)
    unittest.main()
