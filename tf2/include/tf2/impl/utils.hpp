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
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <limits>
#include <cmath>


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

/** The code below is blantantly copied from urdfdom_headers
 * only the normalization has been added.
 * It computes the Euler roll, pitch yaw from a tf2::Quaternion
 * It is equivalent to tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
 * \param q a tf2::Quaternion
 * \param yaw the computed yaw
 * \param pitch the computed pitch
 * \param roll the computed roll
 */
inline
void getEulerYPR(const tf2::Quaternion & q, double & yaw, double & pitch, double & roll)
{
  const double pi_2 = 1.57079632679489661923;
  // Use a larger threshold to handle numerical errors from quaternion computations
  const double epsilon = 1e-10;
  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  // normalization added from urdfom_headers
  double sarg = -2 * (q.x() * q.z() - q.w() * q.y()) / (sqx + sqy + sqz + sqw);
  
  // Apply epsilon thresholding to handle numerical precision issues
  double threshold_high = 0.99999 - epsilon;
  double threshold_low = -0.99999 + epsilon;
  
  if (sarg <= threshold_low) {
    pitch = -0.5 * pi_2;
    roll = 0;
    yaw = -2 * atan2(q.y(), q.x());
  } else if (sarg >= threshold_high) {
    pitch = 0.5 * pi_2;
    roll = 0;
    yaw = 2 * atan2(q.y(), q.x());
  } else {
    pitch = asin(sarg);
    
    // Apply epsilon thresholding to arguments before atan2 calls
    double roll_y = 2 * (q.y() * q.z() + q.w() * q.x());
    double roll_x = sqw - sqx - sqy + sqz;
    double yaw_y = 2 * (q.x() * q.y() + q.w() * q.z());
    double yaw_x = sqw + sqx - sqy - sqz;
    
    // Zero out very small values to prevent atan2 from returning incorrect angles
    if (std::abs(roll_y) < epsilon && std::abs(roll_x) < epsilon) {
      roll = 0;
    } else {
      roll = atan2(roll_y, roll_x);
    }
    
    if (std::abs(yaw_y) < epsilon && std::abs(yaw_x) < epsilon) {
      yaw = 0;
    } else {
      yaw = atan2(yaw_y, yaw_x);
    }
  }
}

/** The code below is a simplified version of getEulerRPY that only
 * returns the yaw. It is mostly useful in navigation where only yaw
 * matters
 * \param q a tf2::Quaternion
 * \return the computed yaw
 */
inline
double getYaw(const tf2::Quaternion & q)
{
  double yaw;
  // Use a larger threshold to handle numerical errors from quaternion computations
  const double epsilon = 1e-10;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  // normalization added from urdfom_headers
  double sarg = -2 * (q.x() * q.z() - q.w() * q.y()) / (sqx + sqy + sqz + sqw);

  // Apply epsilon thresholding to handle numerical precision issues
  double threshold_high = 0.99999 - epsilon;
  double threshold_low = -0.99999 + epsilon;

  if (sarg <= threshold_low) {
    yaw = -2 * atan2(q.y(), q.x());
  } else if (sarg >= threshold_high) {
    yaw = 2 * atan2(q.y(), q.x());
  } else {
    double yaw_y = 2 * (q.x() * q.y() + q.w() * q.z());
    double yaw_x = sqw + sqx - sqy - sqz;
    
    // Zero out very small values to prevent atan2 from returning incorrect angles
    if (std::abs(yaw_y) < epsilon && std::abs(yaw_x) < epsilon) {
      yaw = 0;
    } else {
      yaw = atan2(yaw_y, yaw_x);
    }
  }
  return yaw;
}

}  // namespace impl
}  // namespace tf2
#endif  // TF2__IMPL__UTILS_HPP_
