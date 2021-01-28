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

#ifndef TF2_KDL_H
#define TF2_KDL_H

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
template <typename T, int, int, int, int, int>
class Matrix;
}

namespace tf2
{
/** \brief Convert a timestamped transform to the equivalent KDL data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an KDL Frame.
 */
inline KDL::Frame transformToKDL(const geometry_msgs::msg::TransformStamped & t)
{
  KDL::Frame ans;
  tf2::fromMsg<>(t.transform, ans);
  return ans;
}

/** \brief Convert an KDL Frame to the equivalent geometry_msgs message type.
 * \param k The transform to convert, as an KDL Frame.
 * \return The transform converted to a TransformStamped message.
 */
inline geometry_msgs::msg::TransformStamped kdlToTransform(const KDL::Frame & k)
{
  geometry_msgs::msg::TransformStamped ans;
  tf2::toMsg<>(k, ans.transform);
  return ans;
}

namespace impl
{
template <>
struct ImplDetails<KDL::Frame, geometry_msgs::msg::Transform>
{
  static void fromMsg(geometry_msgs::msg::Transform const & in, KDL::Frame & out)
  {
    KDL::Rotation r;
    KDL::Vector v;
    tf2::fromMsg<>(in.translation, v);
    tf2::fromMsg<>(in.rotation, r);
    out = KDL::Frame(r, v);
  }

  static void toMsg(KDL::Frame const & in, geometry_msgs::msg::Transform & out)
  {
    tf2::toMsg<>(in.p, out.translation);
    tf2::toMsg<>(in.M, out.rotation);
  }
};

template <>
struct ImplDetails<KDL::Vector, geometry_msgs::msg::Vector3>
: DefaultVectorImpl<KDL::Vector, geometry_msgs::msg::Vector3>
{
};

template <>
struct ImplDetails<KDL::Vector, geometry_msgs::msg::Point>
: DefaultVectorImpl<KDL::Vector, geometry_msgs::msg::Point>
{
};

template <>
struct defaultMessage<KDL::Vector>
{
  using type = geometry_msgs::msg::Point;
};

template <>
struct DefaultTransformType<KDL::Vector>
{
  using type = KDL::Frame;
};

// ---------------------
// Twist
// ---------------------

template <>
struct ImplDetails<KDL::Twist, geometry_msgs::msg::Twist>
{
  /** \brief Convert a stamped KDL Twist type to a TwistStamped message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped KDL Twist to convert.
   * \return The twist converted to a TwistStamped message.
   */
  static void toMsg(const KDL::Twist & in, geometry_msgs::msg::Twist & msg)
  {
    tf2::toMsg<>(in.vel, msg.linear);
    tf2::toMsg<>(in.rot, msg.angular);
  }

  /** \brief Convert a TwistStamped message type to a stamped KDL-specific Twist type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The TwistStamped message to convert.
   * \param out The twist converted to a timestamped KDL Twist.
   */
  static void fromMsg(const geometry_msgs::msg::Twist & msg, KDL::Twist & out)
  {
    tf2::fromMsg<>(msg.linear, out.vel);
    tf2::fromMsg<>(msg.angular, out.rot);
  }
};

template <>
struct DefaultTransformType<KDL::Twist>
{
  using type = KDL::Frame;
};

template <>
struct defaultMessage<KDL::Twist>
{
  using type = geometry_msgs::msg::Twist;
};

// ---------------------
// Wrench
// ---------------------

template <>
struct ImplDetails<KDL::Wrench, geometry_msgs::msg::Wrench>
{
  /** \brief Convert a stamped KDL Wrench type to a WrenchStamped message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped KDL Wrench to convert.
   * \return The wrench converted to a WrenchStamped message.
   */
  static void toMsg(const KDL::Wrench & in, geometry_msgs::msg::Wrench & msg)
  {
    tf2::toMsg<>(in.force, msg.force);
    tf2::toMsg<>(in.torque, msg.torque);
  }

  /** \brief Convert a WrenchStamped message type to a stamped KDL-specific Wrench type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The WrenchStamped message to convert.
   * \param out The wrench converted to a timestamped KDL Wrench.
   */
  static void fromMsg(const geometry_msgs::msg::Wrench & msg, KDL::Wrench & out)
  {
    tf2::fromMsg<>(msg.force, out.force);
    tf2::fromMsg<>(msg.torque, out.torque);
  }
};

template <>
struct defaultMessage<KDL::Wrench>
{
  using type = geometry_msgs::msg::Wrench;
};

template <>
struct DefaultTransformType<KDL::Wrench>
{
  using type = KDL::Frame;
};
}  // namespace impl

template <int options, int rows, int cols>
void convert(Eigen::Matrix<double, 6, 1, options, rows, cols> const & in, KDL::Wrench & out)
{
  const auto msg =
    toMsg<Eigen::Matrix<double, 6, 1, options, rows, cols>, geometry_msgs::msg::Wrench>(in);
  fromMsg<>(msg, out);
}

// ---------------------
// Frame
// ---------------------
namespace impl
{
template <>
struct ImplDetails<KDL::Frame, geometry_msgs::msg::Pose>
{
  /** \brief Convert a stamped KDL Frame type to a Pose message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped KDL Frame to convert.
   * \return The frame converted to a Pose message.
   */
  static void toMsg(const KDL::Frame & in, geometry_msgs::msg::Pose & msg)
  {
    tf2::toMsg<>(in.p, msg.position);
    tf2::toMsg<>(in.M, msg.orientation);
  }

  /** \brief Convert a Pose message type to a KDL Frame.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h.
   * \param msg The Pose message to convert.
   * \param out The pose converted to a KDL Frame.
   */
  static void fromMsg(const geometry_msgs::msg::Pose & msg, KDL::Frame & out)
  {
    tf2::fromMsg<>(msg.position, out.p);
    tf2::fromMsg<>(msg.orientation, out.M);
  }
};

template <>
struct defaultMessage<KDL::Frame>
{
  using type = geometry_msgs::msg::Pose;
};

template <>
struct DefaultTransformType<KDL::Frame>
{
  using type = KDL::Frame;
};

template <>
struct ImplDetails<KDL::Rotation, geometry_msgs::msg::Quaternion>
{
  static void toMsg(KDL::Rotation const & r, geometry_msgs::msg::Quaternion & q)
  {
    r.GetQuaternion(q.x, q.y, q.z, q.w);
  }

  static void fromMsg(geometry_msgs::msg::Quaternion const & q, KDL::Rotation & r)
  {
    r = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
  }
};

template <>
struct defaultMessage<KDL::Rotation>
{
  using type = geometry_msgs::msg::Quaternion;
};

}  // namespace impl

/**
 * \brief Transform a KDL::Rotation
 *
 * \param[in] data_in The data to be transformed.
 * \param[in,out] data_out A reference to the output data.
 * \param[in] transform The transform to apply to data_in to fill data_out.
 */
template<>
void doTransform(
  const KDL::Rotation & in, KDL::Rotation & out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Rotation t;
  tf2::fromMsg<>(transform.transform.rotation, t);
  out = t * in;
}

}  // namespace tf2

#endif  // TF2_KDL_H
