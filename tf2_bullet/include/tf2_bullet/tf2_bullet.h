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

/** \author Wim Meeussen */

#ifndef TF2_BULLET__TF2_BULLET_H_
#define TF2_BULLET__TF2_BULLET_H_

#include <tf2/convert.h>
#include <LinearMath/btTransform.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer_interface.h>

#include <iostream>

namespace tf2
{
/** \brief Convert a timestamped transform to the equivalent Bullet data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to a Bullet btTransform.
 */
  inline
  btTransform transformToBullet(const geometry_msgs::msg::TransformStamped & t)
  {
    return btTransform(
      btQuaternion(
        static_cast < float > (t.transform.rotation.x),
        static_cast < float > (t.transform.rotation.y),
        static_cast < float > (t.transform.rotation.z),
        static_cast < float > (t.transform.rotation.w)),
      btVector3(
        static_cast < float > (t.transform.translation.x),
        static_cast < float > (t.transform.translation.y),
        static_cast < float > (t.transform.translation.z)));
  }


/** \brief Apply a geometry_msgs TransformStamped to a Bullet-specific Vector3 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h
 * \param t_in The vector to transform, as a timestamped Bullet btVector3 data type.
 * \param t_out The transformed vector, as a timestamped Bullet btVector3 data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
  template < >
  inline
  void doTransform(
    const tf2::Stamped < btVector3 > & t_in, tf2::Stamped < btVector3 > & t_out,
    const geometry_msgs::msg::TransformStamped & transform)
  {
    t_out =
      tf2::Stamped < btVector3 > (
      transformToBullet(transform) * t_in,
      tf2_ros::fromMsg(transform.header.stamp), transform.header.frame_id);
  }

/** \brief Convert a stamped Bullet Vector3 type to a PointStamped message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h
 * \param in The timestamped Bullet btVector3 to convert.
 * \return The vector converted to a PointStamped message.
 */
  inline
  geometry_msgs::msg::PointStamped toMsg(const tf2::Stamped < btVector3 > & in)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = tf2_ros::toMsg(in.stamp_);
    msg.header.frame_id = in.frame_id_;
    msg.point.x = in[0];
    msg.point.y = in[1];
    msg.point.z = in[2];
    return msg;
  }

/** \brief Convert a PointStamped message type to a stamped Bullet-specific Vector3 type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The PointStamped message to convert.
 * \param out The point converted to a timestamped Bullet Vector3.
 */
  inline
  void fromMsg(const geometry_msgs::msg::PointStamped & msg, tf2::Stamped < btVector3 > & out)
  {
    out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
    out.frame_id_ = msg.header.frame_id;
    out[0] = static_cast < float > (msg.point.x);
    out[1] = static_cast < float > (msg.point.y);
    out[2] = static_cast < float > (msg.point.z);
  }


/** \brief Apply a geometry_msgs TransformStamped to a Bullet-specific Transform data type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h
 * \param t_in The frame to transform, as a timestamped Bullet btTransform.
 * \param t_out The transformed frame, as a timestamped Bullet btTransform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
  template < >
  inline
  void doTransform(
    const tf2::Stamped < btTransform > & t_in, tf2::Stamped < btTransform > & t_out,
    const geometry_msgs::msg::TransformStamped & transform)
  {
    t_out =
      tf2::Stamped < btTransform > (
      transformToBullet(transform) * t_in,
      tf2_ros::fromMsg(transform.header.stamp), transform.header.frame_id);
  }


}  // namespace tf2

#endif  // TF2_BULLET__TF2_BULLET_H_
