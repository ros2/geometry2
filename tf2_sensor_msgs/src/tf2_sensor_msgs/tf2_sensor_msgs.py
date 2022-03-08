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

import tf2_ros
import numpy as np
import numpy.typing as npt
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud, read_points
from geometry_msgs.msg import Transform


def to_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_to_msg(PointCloud2, to_msg_msg)


def from_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_from_msg(PointCloud2, from_msg_msg)


def transform_points(point_cloud: npt.ArrayLike, transform: Transform) -> np.ndarray:
    """
    Transforms a bulk of points from an numpy array using a provided `Transform`.

    :param point_cloud: nx3 Array of points where n is the number of points
    :param transform: TF2 transform used for the transformation
    :returns: Array with the same shape as the input array, but with the transformation applied
    """
    # Cast ArrayLike object to np.ndarray
    point_cloud = np.asarray(point_cloud)

    # Build affine transformation
    transform_translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ])
    transform_rotation_matrix = _get_mat_from_quat(
        np.array([
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z
        ]))

    # "Batched" matmul meaning a matmul for each point
    # First we offset all points by the translation part followed by a rotation using the rotation matrix
    return np.einsum('ij, pj -> pi', transform_rotation_matrix, point_cloud + transform_translation)



def do_transform_cloud(cloud: PointCloud2, transform: Transform) -> PointCloud2:
    """
    Applies a `Transform` on a `PointCloud2`.

    :param cloud: The point cloud that should be transformed
    :param transform: The transform which will applied to the point cloud
    :returns: The transformed point cloud
    """
    points = read_points(cloud)
    points_out = transform_points(points, transform)
    return create_cloud(transform.header, cloud.fields, points_out)


tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Converts a quaternion to a rotation matrix

    This method is currently needed because transforms3d is not released as a `.dep` and
    would require user interaction to set up.

    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: An array containing an X, Y, and Z translation component
    """
    Nq = np.linalg.norm(quaternion)
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [[ 1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
        [ xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
        [ xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])
