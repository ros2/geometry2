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

from typing import Union

from geometry_msgs.msg import Transform, TransformStamped
import numpy as np
try:
    from numpy.lib.recfunctions import (structured_to_unstructured, unstructured_to_structured)
except ImportError:
    # Fix for RHEL because its NumPy version does not include these functions
    from sensor_msgs_py.numpy_compat import (structured_to_unstructured,
                                             unstructured_to_structured)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
import tf2_ros


def to_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_to_msg(PointCloud2, to_msg_msg)


def from_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_from_msg(PointCloud2, from_msg_msg)


def transform_points(
        point_cloud: np.ndarray,
        transform: Transform) -> np.ndarray:
    """
    Transform a bulk of points from an numpy array using a provided `Transform`.

    :param point_cloud: nx3 Array of points where n is the number of points
    :param transform: TF2 transform used for the transformation
    :returns: Array with the same shape as the input array, but with the transformation applied
    """
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
    # First we offset all points by the translation part
    # followed by a rotation using the rotation matrix
    return np.einsum(
        'ij, pj -> pi',
        transform_rotation_matrix,
        point_cloud) + transform_translation


def do_transform_cloud(
        cloud: PointCloud2,
        transform: Union[Transform, TransformStamped]) -> PointCloud2:
    """
    Apply a `Transform` or `TransformStamped` on a `PointCloud2`.

    The x, y, and z values are transformed into a different frame,
    while the rest of the cloud is kept untouched.

    :param cloud: The point cloud that should be transformed
    :param transform: The transform which will applied to the point cloud
    :returns: The transformed point cloud
    """
    # Create new Header so original header is not altered
    new_header = Header(
        stamp=cloud.header.stamp,
        frame_id=cloud.header.frame_id)

    # Check if we have a TransformStamped and are able to update the frame_id
    if isinstance(transform, TransformStamped):
        new_header.frame_id = transform.header.frame_id
        transform = transform.transform

    # Check if xyz are a subset of the field names
    required_fields = set('xyz')
    present_fields = {field.name for field in cloud.fields}
    assert required_fields <= present_fields, \
        'Point cloud needs the fields x, y, and z for the transformation'

    # Read points as structured NumPy array
    points = read_points(cloud)

    # Transform xyz part of the pointcloud using the given transform
    transformed_xyz = transform_points(
        structured_to_unstructured(points[['x', 'y', 'z']]),
        transform)

    # Check if there are additional fields that need to be merged with the transformed coordinates
    if required_fields != present_fields:
        # Merge original array including non coordinate fields with the transformed coordinates
        # The copy is needed as the original message would be altered otherwise
        points = points.copy()
        points[['x', 'y', 'z']] = unstructured_to_structured(transformed_xyz)
    else:
        points = transformed_xyz

    # Serialize pointcloud message
    return create_cloud(new_header, cloud.fields, points)


tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix.

    This method is currently needed because transforms3d is not released as a `.dep` and
    would require user interaction to set up.

    For reference see: https://github.com/matthew-brett/transforms3d/blob/
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101

    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: An array containing an X, Y, and Z translation component
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
