// Copyright (c) 2008, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willo Garage, Inc nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \author Wim Meeussen, Bjarne von Horn */

#ifndef TF2_BULLET__TF2_BULLET_H_
#define TF2_BULLET__TF2_BULLET_H_

#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#if (BT_BULLET_VERSION <= 282)
// Suppress compilation warning on older versions of Bullet.
// TODO(mjcarroll): Remove this when all platforms have the fix upstream.
inline int bullet_btInfinityMask()
{
  return btInfinityMask;
}
#endif

namespace tf2
{
namespace impl
{
/// \brief Default return type of tf2::toMsg() for btVector3.
template<>
struct defaultMessage<btVector3>
{
  /// \brief Default return type of tf2::toMsg() for btVector3.
  using type = geometry_msgs::msg::Point;
};

/// \brief Default return type of tf2::toMsg() for btQuaternion.
template<>
struct defaultMessage<btQuaternion>
{
  /// \brief Default return type of tf2::toMsg() for btQuaternion.
  using type = geometry_msgs::msg::Quaternion;
};

/// \brief Default return type of tf2::toMsg() for btTransform.
template<>
struct defaultMessage<btTransform>
{
  /// \brief Default return type of tf2::toMsg() for btTransform.
  using type = geometry_msgs::msg::Transform;
};

/// \brief Conversion implementation for geometry_msgs::msg::Point and btVector3.
template<>
struct ImplDetails<btVector3, geometry_msgs::msg::Point>
  : DefaultVectorImpl<btVector3, geometry_msgs::msg::Point>
{
};

/// \brief Conversion implementation for geometry_msgs::msg::Vector3 and btVector3.
template<>
struct ImplDetails<btVector3, geometry_msgs::msg::Vector3>
  : DefaultVectorImpl<btVector3, geometry_msgs::msg::Vector3>
{
};

/// \brief Conversion implementation for geometry_msgs::msg::Quaternion and Eigen::Quaterniond.
template<>
struct ImplDetails<btQuaternion, geometry_msgs::msg::Quaternion>
  : DefaultQuaternionImpl<btQuaternion> {};

/// \brief Conversion implementation for geometry_msgs::msg::Transform and btTransform.
template<>
struct ImplDetails<btTransform, geometry_msgs::msg::Transform>
{
  /** \brief Convert a Transform message to a btTransform type.
   * \param[in] in The message to convert.
   * \param[out] out The converted message, as a btTransform type.
   */
  static void fromMsg(geometry_msgs::msg::Transform const & in, btTransform & out)
  {
    btVector3 trans;
    btQuaternion rot;
    tf2::fromMsg<>(in.rotation, rot);
    tf2::fromMsg<>(in.translation, trans);
    out = btTransform(rot, trans);
  }

  /** \brief Convert a btTransform to a Transform message.
   * \param[in] in The btTransform to convert.
   * \param[out] out The converted Transform, as a message.
   */
  static void toMsg(btTransform const & in, geometry_msgs::msg::Transform & out)
  {
    tf2::toMsg<>(in.getRotation(), out.rotation);
    tf2::toMsg<>(in.getOrigin(), out.translation);
  }
};

/// \brief Default Type for automatic tf2::doTransform() implementation for btTransform.
template<>
struct DefaultTransformType<btTransform>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for btTransform.
  using type = btTransform;
};

/// \brief Default Type for automatic tf2::doTransform() implementation for btVector3.
template<>
struct DefaultTransformType<btVector3>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for btVector3.
  using type = btTransform;
};

/// \brief Default Type for automatic tf2::doTransform() implementation for btQuaternion.
template<>
struct DefaultTransformType<btQuaternion>
{
  /// \brief Default Type for automatic tf2::doTransform() implementation for btTransform.
  using type = btTransform;
};
}  // namespace impl

/** \brief Convert a timestamped transform to the equivalent Bullet data type.
 * \param[in] t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to a Bullet btTransform.
 */
inline btTransform transformToBullet(const geometry_msgs::msg::TransformStamped & t)
{
  btTransform ans;
  fromMsg<>(t.transform, ans);
  return ans;
}
}  // namespace tf2

#endif  // TF2_BULLET__TF2_BULLET_H_
