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

from geometry_msgs.msg import (PointStamped, PoseStamped,
                               PoseWithCovarianceStamped, Vector3Stamped)
import numpy as np
import PyKDL
import tf2_ros


def to_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_to_msg(Vector3Stamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PoseStamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PointStamped, to_msg_msg)


def from_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_from_msg(Vector3Stamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PoseStamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PointStamped, from_msg_msg)


def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x,
                                                 t.transform.rotation.y,
                                                 t.transform.rotation.z,
                                                 t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


def transform_covariance(cov_in, transform):
    # Converting the Quaternion to a Rotation Matrix first
    # Taken from: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    q0 = transform.transform.rotation.w
    q1 = transform.transform.rotation.x
    q2 = transform.transform.rotation.y
    q3 = transform.transform.rotation.z

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # Code reference: https://github.com/ros2/geometry2/pull/430
    # Mathematical Reference:
    # A. L. Garcia, “Linear Transformations of Random Vectors,” in Probability,
    # Statistics, and Random Processes For Electrical Engineering, 3rd ed.,
    # Pearson Prentice Hall, 2008, pp. 320–322.

    R = np.array([[r00, r01, r02],
                  [r10, r11, r12],
                  [r20, r21, r22]])

    R_transpose = np.transpose(R)

    cov_11 = np.array([cov_in[:3], cov_in[6:9], cov_in[12:15]])
    cov_12 = np.array([cov_in[3:6], cov_in[9:12], cov_in[15:18]])
    cov_21 = np.array([cov_in[18:21], cov_in[24:27], cov_in[30:33]])
    cov_22 = np.array([cov_in[21:24], cov_in[27:30], cov_in[33:]])

    # And we perform the transform
    result_11 = R @ cov_11 @ R_transpose
    result_12 = R @ cov_12 @ R_transpose
    result_21 = R @ cov_21 @ R_transpose
    result_22 = R @ cov_22 @ R_transpose

    cov_out = PoseWithCovarianceStamped()

    cov_out.pose.covariance[0] = result_11[0][0]
    cov_out.pose.covariance[1] = result_11[0][1]
    cov_out.pose.covariance[2] = result_11[0][2]
    cov_out.pose.covariance[6] = result_11[1][0]
    cov_out.pose.covariance[7] = result_11[1][1]
    cov_out.pose.covariance[8] = result_11[1][2]
    cov_out.pose.covariance[12] = result_11[2][0]
    cov_out.pose.covariance[13] = result_11[2][1]
    cov_out.pose.covariance[14] = result_11[2][2]

    cov_out.pose.covariance[3] = result_12[0][0]
    cov_out.pose.covariance[4] = result_12[0][1]
    cov_out.pose.covariance[5] = result_12[0][2]
    cov_out.pose.covariance[9] = result_12[1][0]
    cov_out.pose.covariance[10] = result_12[1][1]
    cov_out.pose.covariance[11] = result_12[1][2]
    cov_out.pose.covariance[15] = result_12[2][0]
    cov_out.pose.covariance[16] = result_12[2][1]
    cov_out.pose.covariance[17] = result_12[2][2]

    cov_out.pose.covariance[18] = result_21[0][0]
    cov_out.pose.covariance[19] = result_21[0][1]
    cov_out.pose.covariance[20] = result_21[0][2]
    cov_out.pose.covariance[24] = result_21[1][0]
    cov_out.pose.covariance[25] = result_21[1][1]
    cov_out.pose.covariance[26] = result_21[1][2]
    cov_out.pose.covariance[30] = result_21[2][0]
    cov_out.pose.covariance[31] = result_21[2][1]
    cov_out.pose.covariance[32] = result_21[2][2]

    cov_out.pose.covariance[21] = result_22[0][0]
    cov_out.pose.covariance[22] = result_22[0][1]
    cov_out.pose.covariance[23] = result_22[0][2]
    cov_out.pose.covariance[27] = result_22[1][0]
    cov_out.pose.covariance[28] = result_22[1][1]
    cov_out.pose.covariance[29] = result_22[1][2]
    cov_out.pose.covariance[33] = result_22[2][0]
    cov_out.pose.covariance[34] = result_22[2][1]
    cov_out.pose.covariance[35] = result_22[2][2]

    return cov_out.pose.covariance


# PointStamped
def do_transform_point(point, transform):
    p = transform_to_kdl(transform) * PyKDL.Vector(point.point.x,
                                                   point.point.y,
                                                   point.point.z)
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


# Vector3Stamped
def do_transform_vector3(vector3, transform):
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    p = transform_to_kdl(transform) * PyKDL.Vector(vector3.vector.x,
                                                   vector3.vector.y,
                                                   vector3.vector.z)
    res = Vector3Stamped()
    res.vector.x = p[0]
    res.vector.y = p[1]
    res.vector.z = p[2]
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(Vector3Stamped, do_transform_vector3)


# PoseStamped
def do_transform_pose(pose, transform):
    q = PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                  pose.pose.orientation.z, pose.pose.orientation.w)
    vector = PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    f = transform_to_kdl(transform) * PyKDL.Frame(q, vector)
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x, res.pose.orientation.y,
     res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PoseStamped, do_transform_pose)


# PoseWithCovarianceStamped
def do_transform_pose_with_covariance_stamped(pose, transform):
    q = PyKDL.Rotation.Quaternion(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
                                  pose.pose.pose.orientation.z, pose.pose.pose.orientation.w)
    vector = PyKDL.Vector(pose.pose.pose.position.x,
                          pose.pose.pose.position.y,
                          pose.pose.pose.position.z)
    f = transform_to_kdl(transform) * PyKDL.Frame(q, vector)
    res = PoseWithCovarianceStamped()
    res.pose.pose.position.x = f.p[0]
    res.pose.pose.position.y = f.p[1]
    res.pose.pose.position.z = f.p[2]
    (res.pose.pose.orientation.x,
     res.pose.pose.orientation.y,
     res.pose.pose.orientation.z,
     res.pose.pose.orientation.w) = f.M.GetQuaternion()
    res.pose.covariance = transform_covariance(pose.pose.covariance, transform)
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PoseWithCovarianceStamped,
                                    do_transform_pose_with_covariance_stamped)
