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
template <>
struct defaultMessage<btVector3>
{
  using type = geometry_msgs::msg::Point;
};

template <>
struct defaultMessage<btQuaternion>
{
  using type = geometry_msgs::msg::Quaternion;
};

template <>
struct defaultMessage<btTransform>
{
  using type = geometry_msgs::msg::Transform;
};

template <class Message>
struct BulletVectorImpl
{
  /** \brief Convert a stamped Bullet Vector3 type to a PointStamped message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h
   * \param in The timestamped Bullet btVector3 to convert.
   * \return The vector converted to a PointStamped message.
   */
  static void toMsg(const btVector3 & in, Message & msg)
  {
    msg.x = in[0];
    msg.y = in[1];
    msg.z = in[2];
  }

  /** \brief Convert a PointStamped message type to a stamped Bullet-specific Vector3 type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The PointStamped message to convert.
   * \param out The point converted to a timestamped Bullet Vector3.
   */
  static void fromMsg(const Message & msg, btVector3 & out)
  {
    out[0] = msg.x;
    out[1] = msg.y;
    out[2] = msg.z;
  }
};

template <>
struct ImplDetails<btVector3, geometry_msgs::msg::Point>
: BulletVectorImpl<geometry_msgs::msg::Point>
{
};

template <>
struct ImplDetails<btVector3, geometry_msgs::msg::Vector3>
: BulletVectorImpl<geometry_msgs::msg::Vector3>
{
};

template <>
struct ImplDetails<btQuaternion, geometry_msgs::msg::Quaternion>
{
  static void toMsg(const btQuaternion & in, geometry_msgs::msg::Quaternion & msg)
  {
    msg.x = in[0];
    msg.y = in[1];
    msg.z = in[2];
    msg.w = in[3];
  }

  static void fromMsg(const geometry_msgs::msg::Quaternion & msg, btQuaternion & out)
  {
    out[0] = msg.x;
    out[1] = msg.y;
    out[2] = msg.z;
    out[3] = msg.w;
    if (msg.w < 0) out *= -1;
  }
};

template <>
struct ImplDetails<btTransform, geometry_msgs::msg::Transform>
{
  static void fromMsg(geometry_msgs::msg::Transform const & in, btTransform & out)
  {
    btVector3 trans;
    btQuaternion rot;
    tf2::fromMsg<>(in.rotation, rot);
    tf2::fromMsg<>(in.translation, trans);
    out = btTransform(rot, trans);
  }

  static void toMsg(btTransform const & in, geometry_msgs::msg::Transform & out)
  {
    tf2::toMsg<>(in.getRotation(), out.rotation);
    tf2::toMsg<>(in.getOrigin(), out.translation);
  }
};
}  // namespace impl

/** \brief Convert a timestamped transform to the equivalent Bullet data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to a Bullet btTransform.
 */
inline btTransform transformToBullet(const geometry_msgs::msg::TransformStamped & t)
{
  btTransform ans;
  fromMsg<>(t.transform, ans);
  return ans;
}

/** \brief Apply a geometry_msgs TransformStamped to a Bullet-specific Vector3 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h
 * \param t_in The vector to transform, as a timestamped Bullet btVector3 data type.
 * \param t_out The transformed vector, as a timestamped Bullet btVector3 data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const tf2::Stamped<btVector3> & t_in, tf2::Stamped<btVector3> & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = tf2::Stamped<btVector3>(
    transformToBullet(transform) * t_in, transform.header.stamp, transform.header.frame_id);
}

/** \brief Apply a geometry_msgs TransformStamped to a Bullet-specific Transform data type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h
 * \param t_in The frame to transform, as a timestamped Bullet btTransform.
 * \param t_out The transformed frame, as a timestamped Bullet btTransform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const tf2::Stamped<btTransform> & t_in, tf2::Stamped<btTransform> & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = tf2::Stamped<btTransform>(
    transformToBullet(transform) * t_in, transform.header.stamp, transform.header.frame_id);
}

}  // namespace tf2

#endif  // TF2_BULLET__TF2_BULLET_H_
