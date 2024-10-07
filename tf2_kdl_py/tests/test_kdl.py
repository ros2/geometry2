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


import pytest  # noqa: F401

import builtin_interfaces
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, TwistStamped
from geometry_msgs.msg import Vector3Stamped, PointStamped, PoseStamped, Pose, WrenchStamped
import PyKDL
import rclpy
import tf2_ros
from rclpy.time import Time
import tf2_kdl_py  # noqa: F401


class TestKDLConversions:
    def test_transform(self):
        tf_buffer = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation = Vector3(x=2.0, y=4.0, z=6.0)
        t.transform.rotation = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
        t.header.stamp = builtin_interfaces.msg.Time(sec=2)
        t.header.frame_id = 'a'
        t.child_frame_id = 'b'
        tf_buffer.set_transform(t, 'test')
        out = tf_buffer.lookup_transform('a', 'b',
                                         Time(seconds=2),
                                         rclpy.time.Duration(seconds=2))
        assert out.transform.translation.x == 2
        assert out.transform.rotation.x == 1
        assert out.header.frame_id == 'a'
        assert out.child_frame_id == 'b'

        # Vector
        v = PyKDL.Vector(1, 2, 3)
        out = tf_buffer.transform(
            tf2_kdl_py.StampedVector(v, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        assert out.x() == -1
        assert out.y() == 2
        assert out.z() == 3
        assert out.header.stamp.sec == 2

        f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 90, 180), PyKDL.Vector(1, 2, 3))
        out = tf_buffer.transform(
            tf2_kdl_py.StampedFrame(f, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        assert out.p.x() == -1
        assert out.p.y() == 2
        assert out.p.z() == 3
        # TODO(CursedRock17) ensure Rotations are correct
        # assert out.M.GetRPY()[0] == 0
        # assert out.M.GetRPY()[1] == 1
        # assert out.M.GetRPY()[2] == 2
        assert out.header.stamp.sec == 2

        t = PyKDL.Twist(PyKDL.Vector(1, 2, 3), PyKDL.Vector(4, 5, 6))
        out = tf_buffer.transform(
            tf2_kdl_py.StampedTwist(t, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        assert out.vel.x() == 7
        assert out.vel.y() == 10
        assert out.vel.z() == -9
        assert out.rot.x() == 4
        assert out.rot.y() == -5
        assert out.rot.z() == -6
        assert out.header.stamp.sec == 2

        w = PyKDL.Wrench(PyKDL.Vector(1, 2, 3), PyKDL.Vector(4, 5, 6))
        out = tf_buffer.transform(
            tf2_kdl_py.StampedWrench(w, builtin_interfaces.msg.Time(sec=2), 'a'), 'b')
        assert out.force.x() == 1
        assert out.force.y() == -2
        assert out.force.z() == -3
        assert out.torque.x() == 4
        assert out.torque.y() == -5
        assert out.torque.z() == -6
        assert out.header.stamp.sec == 2

    def test_convert_vector(self):
        # Create the PyKDL and geometry_msgs objects
        v = PyKDL.Vector(1, 2, 3)
        vs = tf2_kdl_py.StampedVector(v, builtin_interfaces.msg.Time(sec=1.0), 'a')
        ros_v = PointStamped()
        ros_v.point.x = 1
        ros_v.point.y = 2
        ros_v.point.z = 3
        ros_v.header.stamp = builtin_interfaces.msg.Time(sec=1.0)
        ros_v.header.frame_id = 'a'

        # Test each form of conversion between PyKDL and geometry_msgs types
        vs2 = tf2_ros.convert(vs, tf2_kdl_py.StampedVector)
        vs3 = tf2_ros.convert(vs, PointStamped)
        vs4 = tf2_ros.convert(ros_v, tf2_kdl_py.StampedVector)
        vs5 = tf2_ros.convert(ros_v, PointStamped)

        # Ensure each set of conversions remains constant
        assert vs2.x() == 1
        assert vs2.y() == 2
        assert vs2.z() == 3

        assert vs3.point.x == 1
        assert vs3.point.y == 2
        assert vs3.point.z == 3

        assert vs4.x() == 1
        assert vs4.y() == 2
        assert vs4.z() == 3

        assert vs5.point.x == 1
        assert vs5.point.y == 2
        assert vs5.point.z == 3

    def test_convert_frame(self):
        # Create the PyKDL and geometry_msgs objects
        v = PyKDL.Vector(1, 2, 3)
        r = PyKDL.Rotation.Quaternion(0, 0, 0, 1)
        fs = tf2_kdl_py.StampedFrame(PyKDL.Frame(r, v), builtin_interfaces.msg.Time(sec=1.0), 'a')
        ros_p = PoseStamped()
        ros_p.pose.position.x = 1
        ros_p.pose.position.y = 2
        ros_p.pose.position.z = 3
        ros_p.pose.orientation.x = 0
        ros_p.pose.orientation.y = 0
        ros_p.pose.orientation.z = 0
        ros_p.pose.orientation.w = 1
        ros_p.header.stamp = builtin_interfaces.msg.Time(sec=1.0)
        ros_p.header.frame_id = 'a'

        # Test each form of conversion between PyKDL and geometry_msgs types
        fs2 = tf2_ros.convert(fs, tf2_kdl_py.StampedFrame)
        fs3 = tf2_ros.convert(fs, PoseStamped)
        fs4 = tf2_ros.convert(ros_p, tf2_kdl_py.StampedFrame)
        fs5 = tf2_ros.convert(ros_p, PoseStamped)

        # Ensure each set of conversions remains constant
        assert fs2.p[0] == 1
        assert fs2.p[1] == 2
        assert fs2.p[2] == 3
        assert fs2.M.GetQuaternion()[0] == 0
        assert fs2.M.GetQuaternion()[1] == 0
        assert fs2.M.GetQuaternion()[2] == 0
        assert fs2.M.GetQuaternion()[3] == 1

        assert fs3.pose.position.x == 1
        assert fs3.pose.position.y == 2
        assert fs3.pose.position.z == 3
        assert fs3.pose.orientation.x == 0
        assert fs3.pose.orientation.y == 0
        assert fs3.pose.orientation.z == 0
        assert fs3.pose.orientation.w == 1

        assert fs4.p[0] == 1
        assert fs4.p[1] == 2
        assert fs4.p[2] == 3
        assert fs4.M.GetQuaternion()[0] == 0
        assert fs4.M.GetQuaternion()[1] == 0
        assert fs4.M.GetQuaternion()[2] == 0
        assert fs4.M.GetQuaternion()[3] == 1

        assert fs5.pose.position.x == 1
        assert fs5.pose.position.y == 2
        assert fs5.pose.position.z == 3
        assert fs5.pose.orientation.x == 0
        assert fs5.pose.orientation.y == 0
        assert fs5.pose.orientation.z == 0
        assert fs5.pose.orientation.w == 1

    def test_confert_twist(self):
        # Create the PyKDL and geometry_msgs objects
        v = PyKDL.Vector(1, 2, 3)
        v2 = PyKDL.Vector(1, 2, 3)
        ts = tf2_kdl_py.StampedTwist(PyKDL.Twist(v, v2), builtin_interfaces.msg.Time(sec=1.0), 'a')
        ros_t = TwistStamped()
        ros_t.twist.linear.x = 1
        ros_t.twist.linear.y = 2
        ros_t.twist.linear.z = 3
        ros_t.twist.angular.x = 1
        ros_t.twist.angular.y = 2
        ros_t.twist.angular.z = 3
        ros_t.header.stamp = builtin_interfaces.msg.Time(sec=1.0)
        ros_t.header.frame_id = 'a'

        # Test each form of conversion between PyKDL and geometry_msgs types
        ts2 = tf2_ros.convert(ts, tf2_kdl_py.StampedTwist)
        ts3 = tf2_ros.convert(ts, TwistStamped)
        ts4 = tf2_ros.convert(ros_t, tf2_kdl_py.StampedTwist)
        ts5 = tf2_ros.convert(ros_t, TwistStamped)

        # Ensure each set of conversions remains constant
        assert ts2.vel[0] == 1
        assert ts2.vel[1] == 2
        assert ts2.vel[2] == 3
        assert ts2.rot[0] == 1
        assert ts2.rot[1] == 2
        assert ts2.rot[2] == 3

        assert ts3.twist.linear.x == 1
        assert ts3.twist.linear.y == 2
        assert ts3.twist.linear.z == 3
        assert ts3.twist.angular.x == 1
        assert ts3.twist.angular.y == 2
        assert ts3.twist.angular.z == 3

        assert ts4.vel[0] == 1
        assert ts4.vel[1] == 2
        assert ts4.vel[2] == 3
        assert ts4.rot[0] == 1
        assert ts4.rot[1] == 2
        assert ts4.rot[2] == 3

        assert ts5.twist.linear.x == 1
        assert ts5.twist.linear.y == 2
        assert ts5.twist.linear.z == 3
        assert ts5.twist.angular.x == 1
        assert ts5.twist.angular.y == 2
        assert ts5.twist.angular.z == 3

    def test_convert_wrench(self):
        # Create the PyKDL and geometry_msgs objects
        v = PyKDL.Vector(1, 2, 3)
        v2 = PyKDL.Vector(1, 2, 3)
        w = PyKDL.Wrench(v, v2)
        ws = tf2_kdl_py.StampedWrench(w, builtin_interfaces.msg.Time(sec=1.0), 'a')
        ros_w = WrenchStamped()
        ros_w.wrench.force.x = 1
        ros_w.wrench.force.y = 2
        ros_w.wrench.force.z = 3
        ros_w.wrench.torque.x = 1
        ros_w.wrench.torque.y = 2
        ros_w.wrench.torque.z = 3
        ros_w.header.stamp = builtin_interfaces.msg.Time(sec=1.0)
        ros_w.header.frame_id = 'a'

        # Tesw each form of conversion between PyKDL and geometry_msgs types
        ws2 = tf2_ros.convert(ws, tf2_kdl_py.StampedWrench)
        ws3 = tf2_ros.convert(ws, WrenchStamped)
        ws4 = tf2_ros.convert(ros_w, tf2_kdl_py.StampedWrench)
        ws5 = tf2_ros.convert(ros_w, WrenchStamped)

        # Ensure each set of conversions remains constant
        assert ws2.force[0] == 1
        assert ws2.force[1] == 2
        assert ws2.force[2] == 3
        assert ws2.torque[0] == 1
        assert ws2.torque[1] == 2
        assert ws2.torque[2] == 3

        assert ws3.wrench.force.x == 1
        assert ws3.wrench.force.y == 2
        assert ws3.wrench.force.z == 3
        assert ws3.wrench.torque.x == 1
        assert ws3.wrench.torque.y == 2
        assert ws3.wrench.torque.z == 3

        assert ws4.force[0] == 1
        assert ws4.force[1] == 2
        assert ws4.force[2] == 3
        assert ws4.torque[0] == 1
        assert ws4.torque[1] == 2
        assert ws4.torque[2] == 3

        assert ws5.wrench.force.x == 1
        assert ws5.wrench.force.y == 2
        assert ws5.wrench.force.z == 3
        assert ws5.wrench.torque.x == 1
        assert ws5.wrench.torque.y == 2
        assert ws5.wrench.torque.z == 3
