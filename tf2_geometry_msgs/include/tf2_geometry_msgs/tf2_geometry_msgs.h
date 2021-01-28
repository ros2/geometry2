/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Wim Meeussen, Bjarne von Horn */

#ifndef TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_H_
#define TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_H_

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <kdl/frames.hpp>

#include <array>


namespace Eigen
{
/// \brief Forward declaration
template<typename T, int, int, int, int, int>
class Matrix;
}

namespace tf2
{
/** \brief Convert a TransformStamped message to a KDL frame.
 * \param t TransformStamped message to convert.
 * \return The converted KDL Frame.
 */
inline
KDL::Frame gmTransformToKDL(const geometry_msgs::msg::TransformStamped & t)
{
  return KDL::Frame(
    KDL::Rotation::Quaternion(
      t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z, t.transform.rotation.w),
    KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
}

namespace impl
{
/*************/
/** Vector3 **/
/*************/

/// \brief Conversion implementation for geometry_msgs::msg::Vector3 and tf2::Vector3.
template<>
struct ImplDetails<tf2::Vector3, geometry_msgs::msg::Vector3>
  : DefaultVectorImpl<tf2::Vector3, geometry_msgs::msg::Vector3>
{
};

/// \brief Conversion implementation for geometry_msgs::msg::Point and tf2::Vector3.
template<>
struct ImplDetails<tf2::Vector3, geometry_msgs::msg::Point>
  : DefaultVectorImpl<tf2::Vector3, geometry_msgs::msg::Point>
{
};

/// \brief Default return type of tf2::toMsg() for tf2::Vector3.
template<>
struct defaultMessage<tf2::Vector3>
{
  /// \brief Default return type of tf2::toMsg() for tf2::Vector3.
  using type = geometry_msgs::msg::Vector3;
};

/// \brief Default Type for automatic tf2::doTransform() implementation for tf2::Vector3.
template<>
struct DefaultTransformType<tf2::Vector3>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for tf2::Vector3.
  using type = tf2::Transform;
};

/** \brief Template for geometry_msgs transformation with corresponding tf2 types.
 * This function is used to implement various tf2::doTransform() specializations for
 * geometry_msgs Messages.
 * \param[in] in The message to transform.
 * \param[in,out] out The transformed message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 * \tparam TFDatatype The tf2 datatype which will be used to calculate the transformation.
 * \tparam Message The type of the message to be transformed
 */
template<typename TFDatatype, typename Message>
inline void doTransformWithTf(
  const Message & in, Message & out, const geometry_msgs::msg::TransformStamped & transform)
{
  TFDatatype in_tf, out_tf;
  tf2::convert(in, in_tf);
  tf2::doTransform(in_tf, out_tf, transform);
  tf2::convert(out_tf, out);
}
}  // namespace impl

/********************/
/** Vector3Stamped **/
/********************/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The vector to transform, as a timestamped Vector3 message.
 * \param[in,out] t_out The transformed vector, as a timestamped Vector3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::Vector3 & t_in, geometry_msgs::msg::Vector3 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Vector3 in_tf;
  tf2::Transform transform_tf;
  tf2::convert(t_in, in_tf);
  tf2::convert(transform.transform, transform_tf);
  const tf2::Vector3 out_tf = transform_tf.getBasis() * in_tf;
  tf2::convert(out_tf, t_out);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The vector to transform, as a timestamped Vector3 message.
 * \param[in,out] t_out The transformed vector, as a timestamped Vector3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::Vector3Stamped & t_in, geometry_msgs::msg::Vector3Stamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  doTransform<>(t_in.vector, t_out.vector, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/******************/
/** PointStamped **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The point to transform, as a timestamped Point3 message.
 * \param[in,out] t_out The transformed point, as a timestamped Point3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::PointStamped & t_in, geometry_msgs::msg::PointStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Vector3>(t_in.point, t_out.point, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The point to transform, as a timestamped Point3 message.
 * \param[in,out] t_out The transformed point, as a timestamped Point3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::Point & t_in, geometry_msgs::msg::Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Vector3>(t_in, t_out, transform);
}

/*****************/
/** PoseStamped **/
/*****************/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The pose to transform, as a timestamped Pose3 message.
 * \param[in,out] t_out The transformed pose, as a timestamped Pose3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::PoseStamped & t_in, geometry_msgs::msg::PoseStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Transform>(t_in.pose, t_out.pose, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The pose to transform, as a timestamped Pose3 message.
 * \param[in,out] t_out The transformed pose, as a timestamped Pose3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::Pose & t_in, geometry_msgs::msg::Pose & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Transform>(t_in, t_out, transform);
}

/*******************************/
/** PoseWithCovarianceStamped **/
/*******************************/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The pose to transform, as a timestamped Pose3 message with covariance.
 * \param[in,out] t_out The transformed pose, as a timestamped Pose3 message with covariance.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::PoseWithCovarianceStamped & t_in,
  geometry_msgs::msg::PoseWithCovarianceStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Transform>(t_in.pose.pose, t_out.pose.pose, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
  t_out.pose.covariance = t_in.pose.covariance;
}

/****************/
/** Quaternion **/
/****************/

namespace impl
{
/// \brief Conversion implementation for geometry_msgs::msg::Quaternion and tf2::Quaternion.
template<>
struct ImplDetails<tf2::Quaternion, geometry_msgs::msg::Quaternion>
  : DefaultQuaternionImpl<tf2::Quaternion> {};

/// \brief Default return type of tf2::toMsg() for tf2::Quaternion.
template<>
struct defaultMessage<tf2::Quaternion>
{
  /// \brief Default return type of tf2::toMsg() for tf2::Quaternion.
  using type = geometry_msgs::msg::Quaternion;
};

/// \brief Default Type for automatic tf2::doTransform() implementation for tf2::Quaternion.
template<>
struct DefaultTransformType<tf2::Quaternion>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for tf2::Quaternion.
  using type = tf2::Transform;
};

}  // namespace impl

/***********************/
/** QuaternionStamped **/
/***********************/
/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The quaternion to transform, as a timestamped Quaternion3 message.
 * \param[in,out] t_out The transformed quaternion, as a timestamped Quaternion3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::Quaternion & t_in, geometry_msgs::msg::Quaternion & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Quaternion>(t_in, t_out, transform);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The quaternion to transform, as a timestamped Quaternion3 message.
 * \param[in,out] t_out The transformed quaternion, as a timestamped Quaternion3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::QuaternionStamped & t_in, geometry_msgs::msg::QuaternionStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Quaternion>(t_in.quaternion, t_out.quaternion, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/***************/
/** Transform **/
/***************/

namespace impl
{
/// \brief Conversion implementation for geometry_msgs::msg::Transform and tf2::Transform.
template<>
struct ImplDetails<tf2::Transform, geometry_msgs::msg::Transform>
{
  /** \brief Convert a tf2 Transform type to its equivalent geometry_msgs representation.
   * \param[in] in in A tf2 Transform object.
   * \param[out] out The Transform converted to a geometry_msgs message type.
   */
  static void toMsg(const tf2::Transform & in, geometry_msgs::msg::Transform & out)
  {
    tf2::toMsg<>(in.getOrigin(), out.translation);
    tf2::toMsg<>(in.getRotation(), out.rotation);
  }

  /** \brief Convert a Transform message to its equivalent tf2 representation.
   * \param[in] in A Transform message type.
   * \param[out] out The Transform converted to a tf2 type.
   */
  static void fromMsg(const geometry_msgs::msg::Transform & in, tf2::Transform & out)
  {
    tf2::Vector3 v;
    tf2::fromMsg<>(in.translation, v);
    out.setOrigin(v);
    // w at the end in the constructor
    tf2::Quaternion q;
    tf2::fromMsg<>(in.rotation, q);
    out.setRotation(q);
  }
};

/// \brief Default return type of tf2::toMsg() for tf2::Transform.
template<>
struct defaultMessage<tf2::Transform>
{
  /// \brief Default return type of tf2::toMsg() for tf2::Transform.
  using type = geometry_msgs::msg::Transform;
};
}  // namespace impl

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Transform type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param[in] t_in The frame to transform, as a timestamped Transform3 message.
 * \param[in,out] t_out The frame transform, as a timestamped Transform3 message.
 * \param[in] transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline void doTransform(
  const geometry_msgs::msg::TransformStamped & t_in, geometry_msgs::msg::TransformStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  impl::doTransformWithTf<tf2::Transform>(t_in.transform, t_out.transform, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

namespace impl
{
/**********/
/** Pose **/
/**********/

/// \brief Conversion implementation for geometry_msgs::msg::Pose and tf2::Transform.
template<>
struct ImplDetails<tf2::Transform, geometry_msgs::msg::Pose>
{
  /** \brief Convert a tf2 Transform type to an equivalent geometry_msgs Pose message.
   * \param[in] in A tf2 Transform object.
   * \param[out] out The Transform converted to a geometry_msgs Pose message type.
   */
  static void toMsg(const tf2::Transform & in, geometry_msgs::msg::Pose & out)
  {
    tf2::toMsg<>(in.getOrigin(), out.position);
    tf2::toMsg<>(in.getRotation(), out.orientation);
  }

  /** \brief Convert a geometry_msgs Pose message to an equivalent tf2 Transform type.
   * \param[in] in A Pose message.
   * \param[out] out The Pose converted to a tf2 Transform type.
   */
  static void fromMsg(const geometry_msgs::msg::Pose & in, tf2::Transform & out)
  {
    out.setOrigin(tf2::Vector3(in.position.x, in.position.y, in.position.z));
    // w at the end in the constructor
    out.setRotation(
      tf2::Quaternion(in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w));
  }
};

/// \brief Default Type for automatic tf2::doTransform() implementation for tf2::Transform.
template<>
struct DefaultTransformType<tf2::Transform>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for tf2::Transform.
  using type = tf2::Transform;
};

/************/
/** Wrench **/
/************/

/// \brief Conversion implementation for geometry_msgs::msg::Wrench and tf2::Transform.
template<>
struct ImplDetails<std::array<tf2::Vector3, 2>, geometry_msgs::msg::Wrench>
{
  /** \brief Convert a tf2 Wrench to an equivalent geometry_msgs Wrench message.
   * \param[in] in A tf2 Wrench object.
   * \param[out] out The Wrench converted to a geometry_msgs Wrench message type.
   */
  static void toMsg(const std::array<tf2::Vector3, 2> & in, geometry_msgs::msg::Wrench & out)
  {
    tf2::toMsg<>(std::get<0>(in), out.force);
    tf2::toMsg<>(std::get<1>(in), out.torque);
  }

  /** \brief Convert a geometry_msgs Wrench message to an equivalent tf2 Wrench type.
   * \param[in] msg A Wrench message.
   * \param[out] out The Wrench converted to a tf2 Wrench type.
   */
  static void fromMsg(const geometry_msgs::msg::Wrench & msg, std::array<tf2::Vector3, 2> & out)
  {
    tf2::fromMsg<>(msg.force, std::get<0>(out));
    tf2::fromMsg<>(msg.torque, std::get<1>(out));
  }
};

/// \brief Default return type of tf2::toMsg() for std::array<tf2::Vector3, 2>.
template<>
struct defaultMessage<std::array<tf2::Vector3, 2>>
{
  /// \brief Default return type of tf2::toMsg() for std::array<tf2::Vector3, 2>.
  using type = geometry_msgs::msg::Wrench;
};
}  // namespace impl

/** \brief Specialization of tf2::convert for Eigen::Matrix<double, 6, 1> and std::array<tf2::Vector3, 2>.
 *
 * This specialization of the template defined in tf2/convert.h
 * is for a Wrench represented in an Eigen matrix.
 * To avoid a dependency on Eigen, the Eigen::Matrix template is forward declared.
 * \param[in] in Wrench, as Eigen matrix
 * \param[out] out The Wrench as tf2 type
 * \tparam options Eigen::Matrix template parameter
 * \tparam row Eigen::Matrix template parameter
 * \tparam cols Eigen::Matrix template parameter
 */
template<int options, int row, int cols>
void convert(
  const Eigen::Matrix<double, 6, 1, options, row, cols> & in, std::array<tf2::Vector3, 2> & out)
{
  out[0] = tf2::Vector3{in[0], in[1], in[2]};
  out[1] = tf2::Vector3{in[3], in[4], in[5]};
}

}  // namespace tf2


#endif  // TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_H_
