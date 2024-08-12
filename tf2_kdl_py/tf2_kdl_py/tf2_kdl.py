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


# author: Wim Meeussen

from geometry_msgs.msg import PointStamped, TransformStamped, Pose, PoseStamped
from geometry_msgs.msg import TwistStamped, WrenchStamped
import PyKDL
import tf2_ros
import tf2_kdl_py.transform_datatypes


def transform_to_kdl(t):
    """
    Convert a geometry_msgs TransformStamped message to a PyKDL Frame.

    :param t: The Transform message to convert.
    :type t: geometry_msgs.msg.TransformStamped
    :return: The converted PyKDL frame.
    :rtype: PyKDL.Frame
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x,
                                                 t.transform.rotation.y,
                                                 t.transform.rotation.z,
                                                 t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


def kdl_to_transform(k):
    """
    Convert a PyKDL Frame to the equivalent geometry_msgs message type.

    :param k: The Frame to convert.
    :type t: PyKDL.Frame
    :return: The converted geometry_msgs transform.
    :rtype: geometry_msgs.msg.TransformStamped
    """
    t = TransformStamped()
    t.transform.translation.x = k.p.x()
    t.transform.translation.y = k.p.y()
    t.transform.translation.z = k.p.z()
    k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y,
                      t.transform.rotation.z, t.transform.rotation.w)
    return t

###########
# Vector #
###########


def do_transform_vector(vector_in, transform_in):
    """
    Apply a transform in the form of a geometry_msgs message to a PyKDL vector.

    :param vector_in: The stamped PyKDL vector to transform.
    :type vector_in: tf2_kdl_py.transform_datatypes.StampedVector
    :param transform_in: The transform to apply.
    :type transform_in: geometry_msgs.msg.TransformStamped
    :return: The stamped transformed vector.
    :rtype: tf2_kdl_py.transform_datatypes.StampedVector
    """
    vector_out = transform_to_kdl(transform_in) * vector_in
    return tf2_kdl_py.transform_datatypes.StampedVector(vector_out, transform_in.header.stamp, transform_in.header.frame_id)


tf2_ros.TransformRegistration().add(tf2_kdl_py.transform_datatypes.StampedVector, do_transform_vector)


def to_msg_vector(vector_in):
    """
    Convert a stamped PyKDL Vector to a geometry_msgs PointStamped message.

    :param vector_in: The vector to convert.
    :type vector_in: tf2_kdl_py.transform_datatypes.StampedVector
    :return: The converted vector/point.
    :rtype: geometry_msgs.msg.PointStamped
    """
    msg = PointStamped()
    msg.header.stamp = vector_in.header.stamp
    msg.header.frame_id = vector_in.header.frame_id
    msg.point.x = vector_in.vector[0]
    msg.point.y = vector_in.vector[1]
    msg.point.z = vector_in.vector[2]
    return msg


tf2_ros.ConvertRegistration().add_to_msg(tf2_kdl_py.transform_datatypes.StampedVector, to_msg_vector)


def from_msg_vector(msg):
    """
    Convert a PointStamped message to a stamped PyKDL Vector.

    :param msg: The PointStamped message to convert.
    :type msg: geometry_msgs.msg.PointStamped
    :return: The timestamped converted PyKDL vector.
    :rtype: tf2_kdl_py.transform_datatypes.StampedVector
    """
    vector = PyKDL.Vector(msg.point.x, msg.point.y, msg.point.z)
    return tf2_kdl_py.transform_datatypes.StampedVector(vector, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_from_msg(PointStamped, from_msg_vector)


def convert_vector(vector_in):
    """
    Convert a generic stamped triplet message to a stamped PyKDL Vector.

    :param vector_in: The message to convert.
    :return: The timestamped converted PyKDL vector.
    :rtype: tf2_kdl_py.transform_datatypes.StampedVector(PyKDL.Vector)
    """
    return tf2_kdl_py.transform_datatypes.StampedVetor(PyKDL.Vector(vector_in),
                                vector_in.header.stamp,
                                vector_in.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((PyKDL.Vector, PyKDL.Vector),
                                          convert_vector)


###########
# Frame #
###########


def do_transform_frame(frame_in, transform_in):
    """
    Apply a transform in the form of a geometry_msgs TransformStamped to a PyKDL Frame.

    :param frame_in: The stamped PyKDL frame to transform.
    :type frame_in: tf2_kdl_py.transform_datatypes.StampedFrame
    :param transform_in: The stamped transform to apply.
    :type transform_in: geometry_msgs.msg.TransformStamped
    :return: The stamped transformed PyKDL frame.
    :rtype: tf2_kdl_py.transform_datatypes.StampedFrame
    """
    frame_out = transform_to_kdl(transform_in) * frame_in
    return tf2_kdl_py.transform_datatypes.StampedFrame(frame_out, transform_in.header.stamp, transform_in.header.frame_id)


tf2_ros.TransformRegistration().add(tf2_kdl_py.transform_datatypes.StampedFrame, do_transform_frame)


def to_msg_frame(frame_in):
    """
    Convert a PyKDL Frame to a geometry_msgs Pose message.

    :param frame_in: The frame to convert.
    :type frame_in: PyKDL.Frame
    :return: The converted pose.
    :rtype: geometry_msgs.Pose
    """
    msg = Pose()
    msg.position.x = frame_in.p[0]
    msg.position.y = frame_in.p[1]
    msg.position.z = frame_in.p[2]
    frame_in.M.GetQuaternion(msg.orientation.x, msg.orientation.y,
                             msg.orientation.z, msg.orientation.w)
    return msg


tf2_ros.ConvertRegistration().add_to_msg(PyKDL.Frame, to_msg_frame)


def from_msg_frame(msg):
    """
    Convert a Pose message to a PyKDL frame.

    :param msg: The Pose message to convert.
    :type msg: geometry_msgs.Pose
    :return: The converted PyKDL frame.
    :rtype: PyKDL.Frame
    """
    frame = PyKDL.Frame()
    frame.p[0] = msg.position.x
    frame.p[1] = msg.position.y
    frame.p[2] = msg.position.z
    frame.M = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                        msg.orientation.z, msg.orientation.w)

    return frame


tf2_ros.ConvertRegistration().add_from_msg(PyKDL.Frame, from_msg_frame)


def to_msg_frame_stamped(frame_in):
    """
    Convert a stamped PyKDL Frame to a geometry_msgs PoseStamped message.

    :param frame_in: The stamped frame to convert.
    :type frame_in: tf2_kdl_py.transform_datatypes.StampedFrame
    :return: The converted stamped pose.
    :rtype: geometry_msgs.PoseStamped
    """
    msg = PoseStamped()
    msg.header.stamp = frame_in.header.stamp
    msg.header.frame_id = frame_in.header.frame_id
    msg.pose.position.x = frame_in.frame.p[0]
    msg.pose.position.y = frame_in.frame.p[1]
    msg.pose.position.z = frame_in.frame.p[2]
    orientation = frame_in.frame.M.GetQuaternion()
    msg.pose.orientation.x = orientation[0]
    msg.pose.orientation.y = orientation[1]
    msg.pose.orientation.z = orientation[2]
    msg.pose.orientation.w = orientation[3]
    return msg


tf2_ros.ConvertRegistration().add_to_msg(tf2_kdl_py.transform_datatypes.StampedFrame, to_msg_frame_stamped)


def from_msg_frame_stamped(msg):
    """
    Convert a PoseStamped message to a stamped PyKDL frame.

    :param msg: The Pose message to convert.
    :type msg: geometry_msgs.PoseStamped
    :return: The converted PyKDL stamped frame.
    :rtype: tf2_kdl_py.transform_datatypes.StampedFrame
    """
    rotation = PyKDL.Rotation.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                                         msg.pose.orientation.z, msg.pose.orientation.w)
    vector = PyKDL.Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    frame = PyKDL.Frame(rotation, vector)
    return tf2_kdl_py.transform_datatypes.StampedFrame(frame, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_from_msg(PoseStamped, from_msg_frame_stamped)


###########
# Twist #
###########

def do_transform_twist(twist_in, transform_in):
    """
    Apply a transform in the form of a geometry_msgs TransformStamped to a stamped PyKDL Twist.

    :param twist_in: The stamped PyKDL twist to transform.
    :type twist_in: tf2_kdl_py.transform_datatypes.StampedTwist
    :param transform_in: The transform to apply.
    :type transform_in: geometry_msgs.msg.TransformStamped
    :return: The stamped transformed PyKDL twist.
    :rtype: tf2_kdl_py.transform_datatypes.StampedTwist
    """
    twist_out = transform_to_kdl(transform_in) * twist_in
    return tf2_kdl_py.transform_datatypes.StampedTwist(twist_out, transform_in.header.stamp, transform_in.header.frame_id)


tf2_ros.TransformRegistration().add(tf2_kdl_py.transform_datatypes.StampedTwist, do_transform_twist)


def to_msg_twist_stamped(twist_in):
    """
    Convert a stamped PyKDL Twist to a geometry_msgs TwistStamped message.

    :param twist_in: The stamped twist to convert.
    :type twist_in: tf2_kdl_py.transform_datatypes.StampedTwist
    :return: The converted stamped twist.
    :rtype: geometry_msgs.TwistStamped
    """
    msg = TwistStamped()
    msg.header.stamp = twist_in.header.stamp
    msg.header.frame_id = twist_in.header.frame_id
    msg.twist.linear.x = twist_in.twist.vel[0]
    msg.twist.linear.y = twist_in.twist.vel[1]
    msg.twist.linear.z = twist_in.twist.vel[2]
    msg.twist.angular.x = twist_in.twist.rot[0]
    msg.twist.angular.y = twist_in.twist.rot[1]
    msg.twist.angular.z = twist_in.twist.rot[2]
    return msg


tf2_ros.ConvertRegistration().add_to_msg(tf2_kdl_py.transform_datatypes.StampedTwist, to_msg_twist_stamped)


def from_msg_twist_stamped(msg):
    """
    Convert a TwistStamped message to a stamped PyKDL twist.

    :param msg: The TwistStamped message to convert.
    :type msg: geometry_msgs.TwistStamped
    :return: The converted PyKDL stamped twist.
    :rtype: tf2_kdl_py.transform_datatypes.StampedTwist
    """
    velocity = PyKDL.Vector(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
    rotation = PyKDL.Vector(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)
    twist = PyKDL.Twist(velocity, rotation)
    return tf2_kdl_py.transform_datatypes.StampedTwist(twist, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_from_msg(TwistStamped, from_msg_twist_stamped)


###########
# Wrench #
###########


def do_transform_wrench(wrench_in, transform_in):
    """
    Apply a transform in the form of a geometry_msgs TransformStamped to a PyKDL Wrench.

    :param wrench_in: The Stamped PyKDL wrench to transform.
    :type wrench_in: tf2_kdl_py.transform_datatypes.StampedWrench
    :param transform_in: The transform to apply.
    :type transform_in: geometry_msgs.msg.TransformStamped
    :return: The transformed stamped PyKDL wrench.
    :rtype: tf2_kdl_py.transform_datatypes.StampedWrench
    """
    wrench_out = transform_to_kdl(transform_in) * wrench_in
    return tf2_kdl_py.transform_datatypes.StampedWrench(wrench_out, transform_in.header.stamp, transform_in.header.frame_id)


tf2_ros.TransformRegistration().add(tf2_kdl_py.transform_datatypes.StampedWrench, do_transform_wrench)


def to_msg_wrench_stamped(wrench_in):
    """
    Convert a stamped PyKDL Wrench to a geometry_msgs WrenchStamped message.

    :param wrench_in: The stamped wrench to convert.
    :type wrench_in: tf2_kdl_py.transform_datatypes.StampedWrench
    :return: The converted stamped wrench.
    :rtype: geometry_msgs.WrenchStamped
    """
    msg = WrenchStamped()
    msg.header.stamp = wrench_in.header.stamp
    msg.header.frame_id = wrench_in.header.frame_id
    msg.wrench.force.x = wrench_in.wrench.force[0]
    msg.wrench.force.y = wrench_in.wrench.force[1]
    msg.wrench.force.z = wrench_in.wrench.force[2]
    msg.wrench.torque.x = wrench_in.wrench.torque[0]
    msg.wrench.torque.y = wrench_in.wrench.torque[1]
    msg.wrench.torque.z = wrench_in.wrench.torque[2]
    return msg


tf2_ros.ConvertRegistration().add_to_msg(tf2_kdl_py.transform_datatypes.StampedWrench, to_msg_wrench_stamped)


def from_msg_wrench_stamped(msg):
    """
    Convert a WrenchStamped message to a stamped PyKDL wrench.

    :param msg: The WrenchStamped message to convert.
    :type msg: geometry_msgs.WrenchStamped
    :return: The converted PyKDL stamped wrench.
    :rtype: tf2_kdl_py.transform_datatypes.StampedWrench
    """
    force = PyKDL.Vector(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
    torque = PyKDL.Vector(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)
    wrench = PyKDL.Wrench(force, torque)
    return tf2_kdl_py.transform_datatypes.StampedWrench(wrench, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_from_msg(WrenchStamped, from_msg_wrench_stamped)
