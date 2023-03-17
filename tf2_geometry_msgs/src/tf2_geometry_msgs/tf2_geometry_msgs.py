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

from typing import Iterable, Optional, Tuple

from geometry_msgs.msg import (Point, Point32, PointStamped, PolygonStamped, Pose,
                               PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped, Vector3Stamped)
import numpy as np
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


def transform_covariance(cov_in, transform):
    """
    Apply a given transform to a covariance matrix.

    :param cov_in: Covariance matrix
    :param transform: The transform that will be applies
    :returns: The transformed covariance matrix
    """
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


def _build_affine(
        rotation: Optional[Iterable] = None,
        translation: Optional[Iterable] = None) -> np.ndarray:
    """
    Build an affine matrix from a quaternion and a translation.

    :param rotation: The quaternion as [w, x, y, z]
    :param translation: The translation as [x, y, z]
    :returns: The quaternion and the translation array
    """
    affine = np.eye(4)
    if rotation is not None:
        affine[:3, :3] = _get_mat_from_quat(np.asarray(rotation))
    if translation is not None:
        affine[:3, 3] = np.asarray(translation)
    return affine


def _transform_to_affine(transform: TransformStamped) -> np.ndarray:
    """
    Convert a `TransformStamped` to a affine matrix.

    :param transform: The transform that should be converted
    :returns: The affine transform
    """
    transform = transform.transform
    transform_rotation_matrix = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z
    ]
    transform_translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ]
    return _build_affine(transform_rotation_matrix, transform_translation)


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix.

    This method is based on quat2mat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101 ,
    since that library is not available via rosdep.

    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: The rotation matrix
    """
    Nq = np.sum(np.square(quaternion))
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [[1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
         [xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
         [xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])


def _get_quat_from_mat(rot_mat: np.ndarray) -> np.ndarray:
    """
    Convert a rotation matrix to a quaternion.

    This method is a copy of mat2quat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L150 ,
    since that library is not available via rosdep.

    Method from
    Bar-Itzhack, Itzhack Y. (2000), "New method for extracting the
    quaternion from a rotation matrix", AIAA Journal of Guidance,
    Control and Dynamics 23(6):1085-1087 (Engineering Note), ISSN
    0731-5090

    :param rot_mat: A roatation matrix
    :returns: An quaternion
    """
    # Decompose rotation matrix
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = rot_mat.flat
    # Create matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
    ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector and reorder to w,x,y,z
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Invert quaternion if w is negative (results in positive w)
    if q[0] < 0:
        q *= -1
    return q


def _decompose_affine(affine: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decompose an affine transformation into a quaternion and the translation.

    :param affine: The affine transformation matrix
    :returns: The quaternion and the translation array
    """
    return _get_quat_from_mat(affine[:3, :3]), affine[:3, 3]


# PointStamped
def do_transform_point(
        point: PointStamped,
        transform: TransformStamped) -> PointStamped:
    """
    Transform a `PointStamped` using a given `TransformStamped`.

    :param point: The point
    :param transform: The transform
    :returns: The transformed point
    """
    _, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(translation=[
                point.point.x,
                point.point.y,
                point.point.z
            ])))

    res = PointStamped()
    res.point.x = point[0]
    res.point.y = point[1]
    res.point.z = point[2]
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


# Vector3Stamped
def do_transform_vector3(
        vector3: Vector3Stamped,
        transform: TransformStamped) -> Vector3Stamped:
    """
    Transform a `Vector3Stamped` using a given `TransformStamped`.

    :param vector3: The vector3
    :param transform: The transform
    :returns: The transformed vector3
    """
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    _, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(translation=[
                vector3.vector.x,
                vector3.vector.y,
                vector3.vector.z
            ])))
    res = Vector3Stamped()
    res.vector.x = point[0]
    res.vector.y = point[1]
    res.vector.z = point[2]
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(Vector3Stamped, do_transform_vector3)


# Pose
def do_transform_pose(
        pose: Pose,
        transform: TransformStamped) -> Pose:
    """
    Transform a `Pose` using a given `TransformStamped`.

    This method is used to share the tranformation done in
    `do_transform_pose_stamped()` and `do_transform_pose_with_covariance_stamped()`

    :param pose: The pose
    :param transform: The transform
    :returns: The transformed pose
    """
    quaternion, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(
                translation=[
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ],
                rotation=[
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z])))
    res = Pose()
    res.position.x = point[0]
    res.position.y = point[1]
    res.position.z = point[2]
    res.orientation.w = quaternion[0]
    res.orientation.x = quaternion[1]
    res.orientation.y = quaternion[2]
    res.orientation.z = quaternion[3]
    return res


# PoseStamped
def do_transform_pose_stamped(
        pose: PoseStamped,
        transform: TransformStamped) -> PoseStamped:
    """
    Transform a `PoseStamped` using a given `TransformStamped`.

    :param pose: The stamped pose
    :param transform: The transform
    :returns: The transformed pose stamped
    """
    res = PoseStamped()
    res.pose = do_transform_pose(pose.pose, transform)
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PoseStamped, do_transform_pose_stamped)


# PoseWithCovarianceStamped
def do_transform_pose_with_covariance_stamped(
        pose: PoseWithCovarianceStamped,
        transform: TransformStamped) -> PoseWithCovarianceStamped:
    """
    Transform a `PoseWithCovarianceStamped` using a given `TransformStamped`.

    :param pose: The pose with covariance stamped
    :param transform: The transform
    :returns: The transformed pose with covariance stamped
    """
    res = PoseWithCovarianceStamped()
    res.pose.pose = do_transform_pose(pose.pose.pose, transform)
    res.pose.covariance = transform_covariance(pose.pose.covariance, transform)
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PoseWithCovarianceStamped,
                                    do_transform_pose_with_covariance_stamped)


# PolygonStamped
def do_transform_polygon_stamped(
        polygon: PolygonStamped,
        transform: TransformStamped) -> PolygonStamped:
    """
    Transform a `PolygonStamped` using a given `TransformStamped`.

    :param pose: The polygon stamped
    :param transform: The transform
    :returns: The transformed polygon stamped
    """
    res = PolygonStamped()
    for point in polygon.polygon.points:
        # convert from Point32 to PointStamped
        point = PointStamped(point=Point(x=point.x, y=point.y, z=point.z))
        transformed_point = do_transform_point(point, transform)
        # convert back from PointStamped to Point32
        transformed_point = Point32(x=transformed_point.point.x,
                                    y=transformed_point.point.y,
                                    z=transformed_point.point.z)
        res.polygon.points.append(transformed_point)
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(PolygonStamped,
                                    do_transform_polygon_stamped)
