// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TF2__IMPL__UTILS_HPP_
#define TF2__IMPL__UTILS_HPP_

#include <tf2/convert.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>


namespace tf2
{

// Forward declare functions needed in this header
void fromMsg(const geometry_msgs::msg::Quaternion & in, tf2::Quaternion & out);

namespace impl
{

/** Function needed for the generalization of toQuaternion
 * \param q a tf2::Quaternion
 * \return a copy of the same quaternion
 */
inline
tf2::Quaternion toQuaternion(const tf2::Quaternion & q)
{
  return q;
}

/** Function needed for the generalization of toQuaternion
 * \param q a geometry_msgs::msg::Quaternion
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
inline
tf2::Quaternion toQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion res;
  fromMsg(q, res);
  return res;
}

/** Function needed for the generalization of toQuaternion
 * \param q a geometry_msgs::msg::QuaternionStamped
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
inline
tf2::Quaternion toQuaternion(const geometry_msgs::msg::QuaternionStamped & q)
{
  tf2::Quaternion res;
  fromMsg(q.quaternion, res);
  return res;
}

/** Function needed for the generalization of toQuaternion
 * \param t some tf2::Stamped object
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
template<typename T>
tf2::Quaternion toQuaternion(const tf2::Stamped<T> & t)
{
  geometry_msgs::msg::QuaternionStamped q = toMsg<tf2::Stamped<T>,
      geometry_msgs::msg::QuaternionStamped>(t);
  return toQuaternion(q);
}

/** Generic version of toQuaternion. It tries to convert the argument
 * to a geometry_msgs::msg::Quaternion
 * \param t some object
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
template<typename T>
tf2::Quaternion toQuaternion(const T & t)
{
  geometry_msgs::msg::Quaternion q = toMsg<T, geometry_msgs::msg::QuaternionStamped>(t);
  return toQuaternion(q);
}

/** Computes the Euler roll, pitch yaw from a tf2::Quaternion
 * It internally calls tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
 * \param q a tf2::Quaternion
 * \param yaw the computed yaw
 * \param pitch the computed pitch
 * \param roll the computed roll
 */
inline
void getEulerYPR(const tf2::Quaternion & q, double & yaw, double & pitch, double & roll)
{
  Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
}

/** The code below is the same as getEulerYPR but directly
 * returns the yaw. It is mostly useful in navigation where only yaw
 * matters
 * \param q a tf2::Quaternion
 * \return the computed yaw
 */
inline
double getYaw(const tf2::Quaternion & q)
{
  double yaw;
  double unused_pitch;
  double unused_roll;
  getEulerYPR(q, yaw, unused_pitch, unused_roll);
  return yaw;
}

}  // namespace impl
}  // namespace tf2
#endif  // TF2__IMPL__UTILS_HPP_
