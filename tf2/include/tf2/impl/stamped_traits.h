/*
 * Copyright (c) 2021, Bjarne von Horn
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
 *     * Neither the name of Bjarne von Horn nor the names of its
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

/** \author Bjarne von Horn */


#ifndef TF2_IMPL_STAMPED_TRAITS_H_
#define TF2_IMPL_STAMPED_TRAITS_H_

#include "forward.h"

// forward declarations
namespace geometry_msgs
{
namespace msg
{
template<typename Alloc>
struct Point_;
template<typename Alloc>
struct Vector_;
template<typename Alloc>
struct Quaternion_;
template<typename Alloc>
struct Pose_;
template<typename Alloc>
struct Twist_;
template<typename Alloc>
struct PoseWithCovariance_;
template<typename Alloc>
struct Wrench_;
template<typename Alloc>
struct PointStamped_;
template<typename Alloc>
struct VectorStamped_;
template<typename Alloc>
struct QuaternionStamped_;
template<typename Alloc>
struct PoseStamped_;
template<typename Alloc>
struct TwistStamped_;
template<typename Alloc>
struct PoseWithCovarianceStamped_;
template<typename Alloc>
struct WrenchStamped_;
template<typename Alloc>
struct TransformStamped_;
template<typename Alloc>
struct Transform_;
template<typename Alloc>
struct Vector3_;
template<typename Alloc>
struct Vector3Stamped_;
template<typename Alloc>
struct TwistWithCovarianceStamped_;
}  // namespace msg
}  // namespace geometry_msgs

namespace tf2
{
namespace impl
{

/** \brief Traits for geometry_msgs::msg::Point.
 * \tparam Alloc Message Allocator
*/
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Point_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::PointStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::Vector.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Vector_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::VectorStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::Quaternion.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Quaternion_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::QuaternionStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::Pose.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Pose_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::PoseStamped_<Alloc>;
  /// The corresponding stamped message type with covariance.
  using StampedTypeWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::Twist.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Twist_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::TwistStamped_<Alloc>;
  /// The corresponding stamped message type with covariance.
  using StampedTypeWithCovariance = geometry_msgs::msg::TwistWithCovarianceStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::PoseWithCovariance.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::PoseWithCovariance_<Alloc>>
{
  /// The corresponding stamped message type with covariance.
  using StampedType = geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::Wrench.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Wrench_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::WrenchStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::Transform.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Transform_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::TransformStamped_<Alloc>;
};

/** \brief Traits for geometry_msgs::msg::Vector3.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct UnstampedMessageTraits<geometry_msgs::msg::Vector3_<Alloc>>
{
  /// The corresponding stamped message type.
  using StampedType = geometry_msgs::msg::Vector3Stamped_<Alloc>;
};

/** \brief Template for fast implementation of StampedMessageTraits.
 * \tparam StampedMessage Type of the stamped message.
 * \tparam UnstampedMessage Type of the underlying message.
 * \tparam member Pointer-to-member of the underlying message, e.g. \c &PoseStamped::pose .
 */
template<class StampedMessage, class UnstampedMessage, UnstampedMessage StampedMessage::* member>
struct DefaultStampedMessageTraits
{
  /// The underlying message type.
  using UntampedType = UnstampedMessage;

  /** \brief Read-Write access to the underlying message.
   * \param[in] stamped Reference to the stamped message.
   * \return Reference to the unstamped message.
   */
  static UntampedType & accessMessage(StampedMessage & stamped) {return stamped.*member;}

  /** \brief Read-only access to the underlying message.
   * \param[in] stamped Reference to the stamped message.
   * \return Copy of the unstamped message.
   */
  static UntampedType getMessage(StampedMessage const & stamped) {return stamped.*member;}
};

// we use partial specializations (with the allocator as template parameter)
// to avoid including all the message definitons

/** \brief Traits for geometry_msgs::msg::PointStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::PointStamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::PointStamped_<Alloc>, geometry_msgs::msg::Point_<Alloc>,
    & geometry_msgs::msg::PointStamped_<Alloc>::point>
{
};

/** \brief Traits for geometry_msgs::msg::VectorStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::VectorStamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::VectorStamped_<Alloc>, geometry_msgs::msg::Vector_<Alloc>,
    & geometry_msgs::msg::VectorStamped_<Alloc>::vector>
{
};

/** \brief Traits for geometry_msgs::msg::QuaternionStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::QuaternionStamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::QuaternionStamped_<Alloc>, geometry_msgs::msg::Quaternion_<Alloc>,
    & geometry_msgs::msg::QuaternionStamped_<Alloc>::quaternion>
{
};

/** \brief Traits for geometry_msgs::msg::PoseStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::PoseStamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::PoseStamped_<Alloc>, geometry_msgs::msg::Pose_<Alloc>,
    & geometry_msgs::msg::PoseStamped_<Alloc>::pose>
{
};

/** \brief Traits for geometry_msgs::msg::TwistStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::TwistStamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::TwistStamped_<Alloc>, geometry_msgs::msg::Twist_<Alloc>,
    & geometry_msgs::msg::TwistStamped_<Alloc>::twist>
{
};

/** \brief Traits for geometry_msgs::msg::WrenchStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::WrenchStamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::WrenchStamped_<Alloc>, geometry_msgs::msg::Wrench_<Alloc>,
    & geometry_msgs::msg::WrenchStamped_<Alloc>::wrench>
{
};

/** \brief Traits for geometry_msgs::msg::TransformStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::TransformStamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::TransformStamped_<Alloc>, geometry_msgs::msg::Transform_<Alloc>,
    & geometry_msgs::msg::TransformStamped_<Alloc>::transform>
{
};

/** \brief Traits for geometry_msgs::msg::Vector3Stamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::Vector3Stamped_<Alloc>>
  : DefaultStampedMessageTraits<
    geometry_msgs::msg::Vector3Stamped_<Alloc>, geometry_msgs::msg::Vector3_<Alloc>,
    & geometry_msgs::msg::Vector3Stamped_<Alloc>::vector>
{
};

/** \brief Traits for geometry_msgs::msg::PoseWithCovarianceStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>>
{
  /// The underlying message type.
  using UntampedType = geometry_msgs::msg::Pose_<Alloc>;
  /// The message type itself.
  using StampedTypeWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>;
  /// The covariance type.
  using CovarianceType = std::array<double, 36>;

  /** \brief Read-Write access to the underlying message.
   * \param[in] stamped Reference to the stamped message.
   * \return Reference to the unstamped message.
   */
  static UntampedType & accessMessage(StampedTypeWithCovariance & stamped)
  {
    return stamped.pose.pose;
  }
  /** \brief Read-only access to the underlying message.
   * \param[in] stamped Reference to the stamped message.
   * \return Copy of the unstamped message.
   */
  static UntampedType getMessage(StampedTypeWithCovariance const & stamped)
  {
    return stamped.pose.pose;
  }
  /** \brief Read-Write access to the covariance.
   * \param[in] stamped Reference to the stamped message with covariance.
   * \return Reference to the covariance.
   */
  static CovarianceType & accessCovariance(StampedTypeWithCovariance & stamped)
  {
    return stamped.pose.covariance;
  }
  /** \brief Read-only access to the covariance.
   * \param[in] stamped Reference to the stamped message with covariance.
   * \return Copy of the covariance.
   */
  static CovarianceType getCovariance(StampedTypeWithCovariance const & stamped)
  {
    return stamped.pose.covariance;
  }
};

/** \brief Traits for geometry_msgs::msg::TwistWithCovarianceStamped.
 * \tparam Alloc Message Allocator
 */
template<typename Alloc>
struct StampedMessageTraits<geometry_msgs::msg::TwistWithCovarianceStamped_<Alloc>>
{
  /// The underlying message type.
  using UntampedType = geometry_msgs::msg::Twist_<Alloc>;
  /// The message type itself.
  using StampedTypeWithCovariance = geometry_msgs::msg::TwistWithCovarianceStamped_<Alloc>;
  /// The covariance type.
  using CovarianceType = std::array<double, 36>;

  /** \brief Read-Write access to the underlying message.
   * \param[in] stamped Reference to the stamped message.
   * \return Reference to the unstamped message.
   */
  static UntampedType & accessMessage(StampedTypeWithCovariance & stamped)
  {
    return stamped.twist.twist;
  }
  /** \brief Read-only access to the underlying message.
   * \param[in] stamped Reference to the stamped message.
   * \return Copy of the unstamped message.
   */
  static UntampedType getMessage(StampedTypeWithCovariance const & stamped)
  {
    return stamped.twist.twist;
  }
  /** \brief Read-Write access to the covariance.
   * \param[in] stamped Reference to the stamped message with covariance.
   * \return Reference to the covariance.
   */
  static CovarianceType & accessCovariance(StampedTypeWithCovariance & stamped)
  {
    return stamped.twist.covariance;
  }
  /** \brief Read-only access to the covariance.
   * \param[in] stamped Reference to the stamped message with covariance.
   * \return Copy of the covariance.
   */
  static CovarianceType getCovariance(StampedTypeWithCovariance const & stamped)
  {
    return stamped.twist.covariance;
  }
};

}  // namespace impl
}  // namespace tf2

#endif  // TF2_IMPL_STAMPED_TRAITS_H_
