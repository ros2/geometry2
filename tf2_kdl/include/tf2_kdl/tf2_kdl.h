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

#ifndef TF2_KDL__TF2_KDL_H_
#define TF2_KDL__TF2_KDL_H_

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/frames.hpp>

namespace Eigen
{
/// Forward declaration.
template<typename T, int, int, int, int, int>
class Matrix;
}

namespace tf2
{
/** \brief Convert a timestamped transform to the equivalent KDL data type.
 * \param[in] t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an KDL Frame.
 */
[[deprecated("Please use tf2::fromMsg()")]] inline KDL::Frame
transformToKDL(
  const geometry_msgs::msg::TransformStamped & t)
{
  KDL::Frame ans;
  tf2::fromMsg<>(t.transform, ans);
  return ans;
}

/** \brief Convert an KDL Frame to the equivalent geometry_msgs message type.
 * \param[in] k The transform to convert, as an KDL Frame.
 * \return The transform converted to a TransformStamped message.
 */
[[deprecated("Please use tf2::toMsg()")]] inline geometry_msgs::msg::TransformStamped
kdlToTransform(const KDL::Frame & k)
{
  geometry_msgs::msg::TransformStamped ans;
  tf2::toMsg<>(k, ans.transform);
  return ans;
}

namespace impl
{
// ---------------------
// Vector
// ---------------------


/// \brief Conversion implementation for geometry_msgs::msg::Vector3 and KDL::Vector.
template<>
struct ConversionImplementation<KDL::Vector, geometry_msgs::msg::Vector3>
  : DefaultVectorConversionImplementation<KDL::Vector, geometry_msgs::msg::Vector3>
{
};

/// \brief Conversion implementation for geometry_msgs::msg::Point and KDL::Vector.
template<>
struct ConversionImplementation<KDL::Vector, geometry_msgs::msg::Point>
  : DefaultVectorConversionImplementation<KDL::Vector, geometry_msgs::msg::Point>
{
};

/// \brief Default return type of tf2::toMsg() for KDL::Vector.
template<>
struct DefaultMessageForDatatype<KDL::Vector>
{
  /// \brief Default return type of tf2::toMsg() for KDL::Vector.
  using type = geometry_msgs::msg::Point;
};

/// \brief Default Type for automatic tf2::doTransform() implementation for KDL::Frame.
template<>
struct DefaultTransformType<KDL::Vector>
{
  using type = KDL::Frame;
};


// ---------------------
// Twist
// ---------------------

/// \brief Conversion implementation for geometry_msgs::msg::Twist and KDL::Twist.
template<>
struct ConversionImplementation<KDL::Twist, geometry_msgs::msg::Twist>
{
  /** \brief Convert a KDL Twist type to a Twist message.
   * \param[in] in The KDL Twist to convert.
   * \param[out] msg The twist converted to a Twist message.
   */
  static void toMsg(const KDL::Twist & in, geometry_msgs::msg::Twist & msg)
  {
    tf2::toMsg<>(in.vel, msg.linear);
    tf2::toMsg<>(in.rot, msg.angular);
  }

  /** \brief Convert a Twistmessage type to a KDL-specific Twist type.
   * \param[in] msg The Twist message to convert.
   * \param[out] out The twist converted to a KDL Twist.
   */
  static void fromMsg(const geometry_msgs::msg::Twist & msg, KDL::Twist & out)
  {
    tf2::fromMsg<>(msg.linear, out.vel);
    tf2::fromMsg<>(msg.angular, out.rot);
  }
};

/// \brief Default Type for automatic tf2::doTransform() implementation for KDL::Twist.
template<>
struct DefaultTransformType<KDL::Twist>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for KDL::Twist.
  using type = KDL::Frame;
};

/// \brief Default return type of tf2::toMsg() for KDL::Twist.
template<>
struct DefaultMessageForDatatype<KDL::Twist>
{
  /// \brief Default return type of tf2::toMsg() for KDL::Twist.
  using type = geometry_msgs::msg::Twist;
};


// ---------------------
// Wrench
// ---------------------

/// \brief Conversion implementation for geometry_msgs::msg::Wrench and KDL::Wrench.
template<>
struct ConversionImplementation<KDL::Wrench, geometry_msgs::msg::Wrench>
{
  /** \brief Convert a KDL Wrench type to a Wrench message.
   * \param[in] in The KDL Wrench to convert.
   * \param[out] msg The wrench converted to a Wrench message.
   */
  static void toMsg(const KDL::Wrench & in, geometry_msgs::msg::Wrench & msg)
  {
    tf2::toMsg<>(in.force, msg.force);
    tf2::toMsg<>(in.torque, msg.torque);
  }

  /** \brief Convert a Wrench message type to a KDL-specific Wrench type.
   * \param[in] msg The Wrench message to convert.
   * \param[out] out The wrench converted to a KDL Wrench.
   */
  static void fromMsg(const geometry_msgs::msg::Wrench & msg, KDL::Wrench & out)
  {
    tf2::fromMsg<>(msg.force, out.force);
    tf2::fromMsg<>(msg.torque, out.torque);
  }
};

/// \brief Default return type of tf2::toMsg() for KDL::Wrench.
template<>
struct DefaultMessageForDatatype<KDL::Wrench>
{
  /// \brief Default return type of tf2::toMsg() for KDL::Wrench.
  using type = geometry_msgs::msg::Wrench;
};

/// \brief Default Type for automatic tf2::doTransform() implementation for KDL::Wrench.
template<>
struct DefaultTransformType<KDL::Wrench>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for KDL::Wrench.
  using type = KDL::Frame;
};


// ---------------------
// Frame
// ---------------------

/// \brief Conversion implementation for geometry_msgs::msg::Transform and KDL::Frame.
template<>
struct ConversionImplementation<KDL::Frame, geometry_msgs::msg::Transform>
{
  /** \brief Convert a Transform message type to a KDL Frame.
   * \param[in] in The Transform message to convert.
   * \param[out] out The transform converted to a KDL Frame.
   */
  static void fromMsg(geometry_msgs::msg::Transform const & in, KDL::Frame & out)
  {
    KDL::Rotation r;
    KDL::Vector v;
    tf2::fromMsg<>(in.translation, v);
    tf2::fromMsg<>(in.rotation, r);
    out = KDL::Frame(r, v);
  }

  /** \brief Convert a KDL Frame type to a Transform message.
   * \param[in] in The KDL Frame to convert.
   * \param[out] out The KDL Frame converted to a Transform message.
   */
  static void toMsg(KDL::Frame const & in, geometry_msgs::msg::Transform & out)
  {
    tf2::toMsg<>(in.p, out.translation);
    tf2::toMsg<>(in.M, out.rotation);
  }
};

/// \brief Conversion implementation for geometry_msgs::msg::Pose and KDL::Frame.
template<>
struct ConversionImplementation<KDL::Frame, geometry_msgs::msg::Pose>
{
  /** \brief Convert a KDL Frame type to a Pose message.
   * \param[in] in The KDL Frame to convert.
   * \param[out] msg The frame converted to a Pose message.
   */
  static void toMsg(const KDL::Frame & in, geometry_msgs::msg::Pose & msg)
  {
    tf2::toMsg<>(in.p, msg.position);
    tf2::toMsg<>(in.M, msg.orientation);
  }

  /** \brief Convert a Pose message type to a KDL Frame.
   * \param[in] msg The Pose message to convert.
   * \param[out] out The pose converted to a KDL Frame.
   */
  static void fromMsg(const geometry_msgs::msg::Pose & msg, KDL::Frame & out)
  {
    tf2::fromMsg<>(msg.position, out.p);
    tf2::fromMsg<>(msg.orientation, out.M);
  }
};

/// \brief Default return type of tf2::toMsg() for KDL::Frame.
template<>
struct DefaultMessageForDatatype<KDL::Frame>
{
  /// \brief Default return type of tf2::toMsg() for KDL::Frame.
  using type = geometry_msgs::msg::Pose;
};

/// \brief Default Type for automatic tf2::doTransform() implementation for KDL::Frame.
template<>
struct DefaultTransformType<KDL::Frame>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for KDL::Frame.
  using type = KDL::Frame;
};


// ---------------------
// Rotation
// ---------------------

/// \brief Conversion implementation for geometry_msgs::msg::Quaternion and KDL::Rotation.
template<>
struct ConversionImplementation<KDL::Rotation, geometry_msgs::msg::Quaternion>
{
  /** \brief Convert a KDL Rotation type to a Quaternion message.
   * \param[in] r The KDL Rotation to convert.
   * \param[out] q The frame converted to a Quaternion message.
   */
  static void toMsg(KDL::Rotation const & r, geometry_msgs::msg::Quaternion & q)
  {
    r.GetQuaternion(q.x, q.y, q.z, q.w);
  }

  /** \brief Convert a Quaternion message type to a KDL Rotation.
   * \param[in] q The Quaternion message to convert.
   * \param[out] r The quaternion converted to a KDL Rotation.
   */
  static void fromMsg(geometry_msgs::msg::Quaternion const & q, KDL::Rotation & r)
  {
    r = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
  }
};

/// \brief Default return type of tf2::toMsg() for KDL::Rotation.
template<>
struct DefaultMessageForDatatype<KDL::Rotation>
{
  /// \brief Default return type of tf2::toMsg() for KDL::Rotation.
  using type = geometry_msgs::msg::Quaternion;
};

}  // namespace impl

/**
 * \brief Transform a KDL::Rotation
 *
 * \param[in] in The data to be transformed.
 * \param[in,out] out A reference to the output data.
 * \param[in] transform The transform to apply to data_in to fill data_out.
 */
template<>
inline
void doTransform(
  const KDL::Rotation & in, KDL::Rotation & out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Rotation t;
  tf2::fromMsg<>(transform.transform.rotation, t);
  out = t * in;
}

/** \brief Specialization of tf2::convert for Eigen::Matrix<double, 6, 1> and KDL::Wrench.
 *
 * This specialization of the template defined in tf2/convert.h
 * is for a Wrench represented in an Eigen matrix.
 * To avoid a dependency on Eigen, the Eigen::Matrix template is forward declared.
 * \param[in] in Wrench, as Eigen matrix
 * \param[out] out The Wrench as KDL::Wrench type
 * \tparam options Eigen::Matrix template parameter
 * \tparam row Eigen::Matrix template parameter
 * \tparam cols Eigen::Matrix template parameter
 */
template<int options, int rows, int cols>
void convert(Eigen::Matrix<double, 6, 1, options, rows, cols> const & in, KDL::Wrench & out)
{
  const auto msg =
    toMsg<Eigen::Matrix<double, 6, 1, options, rows, cols>, geometry_msgs::msg::Wrench>(in);
  fromMsg<>(msg, out);
}
}  // namespace tf2

#endif  // TF2_KDL__TF2_KDL_H_
