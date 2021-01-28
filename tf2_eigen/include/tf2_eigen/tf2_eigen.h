/*
 * Copyright (c) Koji Terada
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

/** \author Koji Terada, Bjarne von Horn */

#ifndef TF2_EIGEN__TF2_EIGEN_H_
#define TF2_EIGEN__TF2_EIGEN_H_

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace tf2
{
/** \brief Convert a timestamped transform to the equivalent Eigen data type.
 * \param t The transform to convert, as a geometry_msgs Transform message.
 * \return The transform message converted to an Eigen Isometry3d transform.
 */
inline
Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Isometry3d res;
  tf2::fromMsg<>(t, res);
  return res;
}

/** \brief Convert a timestamped transform to the equivalent Eigen data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an Eigen Isometry3d transform.
 */
inline
Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::TransformStamped & t)
{
  return transformToEigen(t.transform);
}

/** \brief Convert an Eigen Affine3d transform to the equivalent geometry_msgs message type.
 * \param T The transform to convert, as an Eigen Affine3d transform.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::msg::TransformStamped eigenToTransform(const Eigen::Affine3d & T)
{
  geometry_msgs::msg::TransformStamped t;
  tf2::toMsg<>(T, t.transform);
  return t;
}

/** \brief Convert an Eigen Isometry3d transform to the equivalent geometry_msgs message type.
 * \param T The transform to convert, as an Eigen Isometry3d transform.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::msg::TransformStamped eigenToTransform(const Eigen::Isometry3d & T)
{
  geometry_msgs::msg::TransformStamped t;
  tf2::toMsg<>(T, t.transform);
  return t;
}

namespace impl
{
/** \brief Template for Eigen::Isometry3d and Eigen::Affine3d conversion
 * \tparam mode Mode argument for Eigen::Transform template
 */
template<int mode>
struct EigenTransformImpl
{
  /** \brief Convert a Transform message to an Eigen type.
   * \param msg The message to convert
   * \param out The converted message, as an Eigen type
   */
  static void fromMsg(
    const geometry_msgs::msg::Transform & msg, Eigen::Transform<double, 3, mode> & out)
  {
    Eigen::Quaterniond q;
    Eigen::Vector3d v;
    tf2::fromMsg<>(msg.rotation, q);
    tf2::fromMsg<>(msg.translation, v);
    out = Eigen::Translation3d(v) * q;
  }

  /** \brief Convert an Eigen Transform to a Transform message
   * \param in The Eigen Transform to convert
   * \param msg The converted Transform, as a message
   */
  static void toMsg(
    const Eigen::Transform<double, 3, mode> & in, geometry_msgs::msg::Transform & msg)
  {
    tf2::toMsg<>(Eigen::Quaterniond(in.linear()), msg.rotation);
    tf2::toMsg<>(Eigen::Vector3d(in.translation()), msg.translation);
  }
};

/// \brief Conversion implementation for geometry_msgs::msg::Transform and Eigen::Affine3d
template<>
struct ImplDetails<Eigen::Affine3d, geometry_msgs::msg::Transform>
  : EigenTransformImpl<Eigen::Affine>
{
};

/// \brief Conversion implementation for geometry_msgs::msg::Transform and Eigen::Isometry3d
template<>
struct ImplDetails<Eigen::Isometry3d, geometry_msgs::msg::Transform>
  : EigenTransformImpl<Eigen::Isometry>
{
};

/// \brief Conversion implementation for geometry_msgs::msg::Point and Eigen::Vector3d
template<>
struct ImplDetails<Eigen::Vector3d, geometry_msgs::msg::Point>
  : DefaultVectorImpl<Eigen::Vector3d, geometry_msgs::msg::Point>
{
};

/// \brief Conversion implementation for geometry_msgs::msg::Vector3d and Eigen::Vector3d
template<>
struct ImplDetails<Eigen::Vector3d, geometry_msgs::msg::Vector3>
  : DefaultVectorImpl<Eigen::Vector3d, geometry_msgs::msg::Vector3>
{
};


template<>
struct DefaultTransformType<Eigen::Vector3d>
{
  using type = Eigen::Isometry3d;
};

template <>
struct defaultMessage<Eigen::Vector3d>
{
  using type = geometry_msgs::msg::Point;
};

/// \brief Conversion implementation for geometry_msgs::msg::Quaternion and Eigen::Quaterniond
template<>
struct ImplDetails<Eigen::Quaterniond, geometry_msgs::msg::Quaternion>
{

  /** \brief Convert a Eigen Quaterniond type to a Quaternion message.
   * \param in The Eigen Quaterniond to convert.
   * \param msg The quaternion converted to a Quaterion message.
   */
  static void toMsg(const Eigen::Quaterniond & in, geometry_msgs::msg::Quaternion & msg)
  {
    msg.w = in.w();
    msg.x = in.x();
    msg.y = in.y();
    msg.z = in.z();
  }

  /** \brief Convert a Quaternion message type to a Eigen-specific Quaterniond type.
   * \param msg The Quaternion message to convert.
   * \param out The quaternion converted to a Eigen Quaterniond.
   */
  static void fromMsg(const geometry_msgs::msg::Quaternion & msg, Eigen::Quaterniond & out)
  {
    out = Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
  }
};

template <>
struct defaultMessage<Eigen::Quaterniond>
{
  using type = geometry_msgs::msg::Quaternion;
};
}  // namespace impl

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Quaterniond type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * functions rely on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The vector to transform, as a Eigen Quaterniond data type.
 * \param t_out The transformed vector, as a Eigen Quaterniond data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const Eigen::Quaterniond & t_in,
  Eigen::Quaterniond & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::Quaterniond t;
  fromMsg(transform.transform.rotation, t);
  t_out = t * t_in;
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Quaterniond type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Eigen Quaterniond data type.
 * \param t_out The transformed vector, as a timestamped Eigen Quaterniond data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const tf2::Stamped<Eigen::Quaterniond> & t_in,
  tf2::Stamped<Eigen::Quaterniond> & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out.frame_id_ = transform.header.frame_id;
  tf2::fromMsg(transform.header.stamp, t_out.stamp_);
  doTransform(
    static_cast<const Eigen::Quaterniond &>(t_in),
    static_cast<Eigen::Quaterniond &>(t_out), transform);
}

namespace impl
{
template <typename T>
struct PoseImplDetails
{
  /** \brief Convert a Eigen Affine3d transform type to a Pose message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The Eigen Affine3d to convert.
   * \return The Eigen transform converted to a Pose message.
   */
  static void toMsg(const T & in, geometry_msgs::msg::Pose & msg)
  {
    tf2::toMsg<>(Eigen::Vector3d(in.translation()), msg.position);
    Eigen::Quaterniond q(in.linear());
    if (q.w() < 0) q.coeffs() *= -1;
    tf2::toMsg<>(q, msg.orientation);
  }

  /** \brief Convert a Pose message transform type to a Eigen Affine3d.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param msg The Pose message to convert.
   * \param out The pose converted to a Eigen Affine3d.
   */
  static void fromMsg(const geometry_msgs::msg::Pose & msg, T & out)
  {
    Eigen::Quaterniond q;
    Eigen::Vector3d v;
    tf2::fromMsg<>(msg.orientation, q);
    tf2::fromMsg<>(msg.position, v);
    out = Eigen::Translation3d(v) * q;
  }
};

template <>
struct ImplDetails<Eigen::Affine3d, geometry_msgs::msg::Pose>
: public PoseImplDetails<Eigen::Affine3d>
{
};

template <>
struct ImplDetails<Eigen::Isometry3d, geometry_msgs::msg::Pose>
: public PoseImplDetails<Eigen::Isometry3d>
{
};

template <>
struct defaultMessage<Eigen::Affine3d>
{
  using type = geometry_msgs::msg::Pose;
};

template <>
struct defaultMessage<Eigen::Isometry3d>
{
  using type = geometry_msgs::msg::Pose;
};
/** \brief Convert a Eigen 6x1 Matrix type to a Twist message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The 6x1 Eigen Matrix to convert.
 * \return The Eigen Matrix converted to a Twist message.
 */
template <>
struct ImplDetails<Eigen::Matrix<double, 6, 1>, geometry_msgs::msg::Twist>
{
  static void toMsg(const Eigen::Matrix<double, 6, 1> & in, geometry_msgs::msg::Twist & msg)
  {
    msg.linear.x = in[0];
    msg.linear.y = in[1];
    msg.linear.z = in[2];
    msg.angular.x = in[3];
    msg.angular.y = in[4];
    msg.angular.z = in[5];
  }

  /** \brief Convert a Twist message transform type to a Eigen 6x1 Matrix.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param msg The Twist message to convert.
   * \param out The twist converted to a Eigen 6x1 Matrix.
   */
  static void fromMsg(const geometry_msgs::msg::Twist & msg, Eigen::Matrix<double, 6, 1> & out)
  {
    out[0] = msg.linear.x;
    out[1] = msg.linear.y;
    out[2] = msg.linear.z;
    out[3] = msg.angular.x;
    out[4] = msg.angular.y;
    out[5] = msg.angular.z;
  }
};

template <>
struct defaultMessage<Eigen::Matrix<double, 6, 1>>
{
  using type = geometry_msgs::msg::Twist;
};

template <>
struct ImplDetails<Eigen::Matrix<double, 6, 1>, geometry_msgs::msg::Wrench>
{
  static void toMsg(const Eigen::Matrix<double, 6, 1> & in, geometry_msgs::msg::Wrench & msg)
  {
    msg.force.x = in[0];
    msg.force.y = in[1];
    msg.force.z = in[2];
    msg.torque.x = in[3];
    msg.torque.y = in[4];
    msg.torque.z = in[5];
  }

  /** \brief Convert a Twist message transform type to a Eigen 6x1 Matrix.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param msg The Twist message to convert.
   * \param out The twist converted to a Eigen 6x1 Matrix.
   */
  static void fromMsg(const geometry_msgs::msg::Wrench & msg, Eigen::Matrix<double, 6, 1> & out)
  {
    out[0] = msg.force.x;
    out[1] = msg.force.y;
    out[2] = msg.force.z;
    out[3] = msg.torque.x;
    out[4] = msg.torque.y;
    out[5] = msg.torque.z;
  }
};

template<>
struct DefaultTransformType<Eigen::Affine3d>
{
  using type = Eigen::Isometry3d;
};


template<>
struct DefaultTransformType<Eigen::Isometry3d>
{
  using type = Eigen::Isometry3d;
};

}  // namespace impl

/** \brief Convert a Eigen::Vector3d type to a geometry_msgs::msg::Vector3.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * The function was renamed to toMsg2 to avoid overloading conflicts with definitions above.
 *
 * \param in The Eigen::Vector3d to convert.
 * \return The Eigen::Vector3d converted to a geometry_msgs::msg::Vector3 message.
 */
inline
geometry_msgs::msg::Vector3 toMsg2(const Eigen::Vector3d & in)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = in(0);
  msg.y = in(1);
  msg.z = in(2);
  return msg;
}
}  // namespace tf2

#endif  // TF2_EIGEN_H
