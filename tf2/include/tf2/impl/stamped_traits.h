
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
template <typename Alloc>
class Point_;
template <typename Alloc>
class Vector_;
template <typename Alloc>
class Quaternion_;
template <typename Alloc>
class Pose_;
template <typename Alloc>
class Twist_;
template <typename Alloc>
class PoseWithCovariance_;
template <typename Alloc>
class Wrench_;
template <typename Alloc>
class PointStamped_;
template <typename Alloc>
class VectorStamped_;
template <typename Alloc>
class QuaternionStamped_;
template <typename Alloc>
class PoseStamped_;
template <typename Alloc>
class TwistStamped_;
template <typename Alloc>
class PoseWithCovarianceStamped_;
template <typename Alloc>
class WrenchStamped_;
template <typename Alloc>
class TransformStamped_;
template <typename Alloc>
class Transform_;
template <typename Alloc>
class Vector3_;
template <typename Alloc>
class Vector3Stamped_;
}  // namespace msg
}  // namespace geometry_msgs

namespace tf2
{
namespace impl
{

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Point_<Alloc>>
{
  using stampedType = geometry_msgs::msg::PointStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Vector_<Alloc>>
{
  using stampedType = geometry_msgs::msg::VectorStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Quaternion_<Alloc>>
{
  using stampedType = geometry_msgs::msg::QuaternionStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Pose_<Alloc>>
{
  using stampedType = geometry_msgs::msg::PoseStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Twist_<Alloc>>
{
  using stampedType = geometry_msgs::msg::TwistStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::PoseWithCovariance_<Alloc>>
{
  using stampedType = geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Wrench_<Alloc>>
{
  using stampedType = geometry_msgs::msg::WrenchStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Transform_<Alloc>>
{
  using stampedType = geometry_msgs::msg::TransformStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::msg::Vector3_<Alloc>>
{
  using stampedType = geometry_msgs::msg::Vector3Stamped_<Alloc>;
};

template <class StampedMessage, class UnstampedMessage, UnstampedMessage StampedMessage::*member>
struct defaultStampedMessageTraits
{
  using unstampedType = UnstampedMessage;
  static unstampedType & accessMessage(StampedMessage & stamped) { return stamped.*member; }
  static unstampedType getMessage(StampedMessage const & stamped) { return stamped.*member; }
};

// we use partial specializations (with the allocator as template parameter)
// to avoid including all the message definitons

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::PointStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::PointStamped_<Alloc>, geometry_msgs::msg::Point_<Alloc>,
    &geometry_msgs::msg::PointStamped_<Alloc>::point>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::VectorStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::VectorStamped_<Alloc>, geometry_msgs::msg::Vector_<Alloc>,
    &geometry_msgs::msg::VectorStamped_<Alloc>::vector>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::QuaternionStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::QuaternionStamped_<Alloc>, geometry_msgs::msg::Quaternion_<Alloc>,
    &geometry_msgs::msg::QuaternionStamped_<Alloc>::quaternion>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::PoseStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::PoseStamped_<Alloc>, geometry_msgs::msg::Pose_<Alloc>,
    &geometry_msgs::msg::PoseStamped_<Alloc>::pose>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::TwistStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::TwistStamped_<Alloc>, geometry_msgs::msg::Twist_<Alloc>,
    &geometry_msgs::msg::TwistStamped_<Alloc>::twist>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>,
    geometry_msgs::msg::PoseWithCovariance_<Alloc>,
    &geometry_msgs::msg::PoseWithCovarianceStamped_<Alloc>::pose>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::WrenchStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::WrenchStamped_<Alloc>, geometry_msgs::msg::Wrench_<Alloc>,
    &geometry_msgs::msg::WrenchStamped_<Alloc>::wrench>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::TransformStamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::TransformStamped_<Alloc>, geometry_msgs::msg::Transform_<Alloc>,
    &geometry_msgs::msg::TransformStamped_<Alloc>::transform>
{
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::msg::Vector3Stamped_<Alloc>>
: defaultStampedMessageTraits<
    geometry_msgs::msg::Vector3Stamped_<Alloc>, geometry_msgs::msg::Vector3_<Alloc>,
    &geometry_msgs::msg::Vector3Stamped_<Alloc>::vector>
{
};

}  // namespace impl
}  // namespace tf2

#endif  // TF2_IMPL_STAMPED_TRAITS_H_
