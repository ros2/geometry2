// Copyright 2008 Willow Garage, Inc.
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
//    * Neither the name of the Willow Garage, Inc. nor the names of its
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

#ifndef TF2_SENSOR_MSGS__TF2_SENSOR_MSGS_HPP_
#define TF2_SENSOR_MSGS__TF2_SENSOR_MSGS_HPP_

#include <string>

// Version 3.4.0 of Eigen in Ubuntu 22.04 has a bug that causes -Wclass-memaccess warnings on
// aarch64.  Upstream Eigen has already fixed this in
// https://gitlab.com/libeigen/eigen/-/merge_requests/645 .  The Debian fix for this is in
// https://salsa.debian.org/science-team/eigen3/-/merge_requests/1 .
// However, it is not clear that that fix is going to make it into Ubuntu 22.04 before it
// freezes, so disable the warning here.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
#include <Eigen/Eigen>  // NOLINT
#include <Eigen/Geometry>  // NOLINT
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include "tf2_ros/buffer_interface.hpp"

#include "tf2/convert.hpp"
#include "tf2/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace tf2
{

/********************/
/** PointCloud2    **/
/********************/

// method to extract timestamp from object
template<>
inline
tf2::TimePoint getTimestamp(const sensor_msgs::msg::PointCloud2 & p)
{
  return tf2_ros::fromMsg(p.header.stamp);
}

// method to extract frame id from object
template<>
inline
std::string getFrameId(const sensor_msgs::msg::PointCloud2 & p) {return p.header.frame_id;}

// this method needs to be implemented by client library developers
template<>
inline
void doTransform(
  const sensor_msgs::msg::PointCloud2 & p_in, sensor_msgs::msg::PointCloud2 & p_out,
  const geometry_msgs::msg::TransformStamped & t_in)
{
  bool normals = std::find_if(
    p_in.fields.begin(),
    p_in.fields.end(),
    [](const sensor_msgs::msg::PointField & fname) {
      return fname.name == std::string("normal_x");
    }) != p_in.fields.end();

  p_out = p_in;
  p_out.header = t_in.header;
  // FIXME(clalancette): The static casts to float aren't ideal; the incoming
  // transform uses double, and hence may have values that are too large to represent
  // in a float.  But since this method implicitly returns a float PointCloud2, we don't
  // have much choice here without subtly changing the API.
  auto translation = Eigen::Translation3f(
    static_cast<float>(t_in.transform.translation.x),
    static_cast<float>(t_in.transform.translation.y),
    static_cast<float>(t_in.transform.translation.z));
  auto quaternion = Eigen::Quaternion<float>(
    static_cast<float>(t_in.transform.rotation.w),
    static_cast<float>(t_in.transform.rotation.x),
    static_cast<float>(t_in.transform.rotation.y),
    static_cast<float>(t_in.transform.rotation.z));

  Eigen::Transform<float, 3, Eigen::Affine> t = translation * quaternion;
  Eigen::Transform<float, 3, Eigen::Affine> r = Eigen::Translation3f(0, 0, 0) * quaternion;

  sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, std::string("x"));
  sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, std::string("y"));
  sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, std::string("z"));

  sensor_msgs::PointCloud2Iterator<float> x_out(p_out, std::string("x"));
  sensor_msgs::PointCloud2Iterator<float> y_out(p_out, std::string("y"));
  sensor_msgs::PointCloud2Iterator<float> z_out(p_out, std::string("z"));

  Eigen::Vector3f point;

  if (!normals) {
    for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
      point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);
      *x_out = point.x();
      *y_out = point.y();
      *z_out = point.z();
    }
  } else {
    Eigen::Vector3f normal;
    sensor_msgs::PointCloud2ConstIterator<float> x_n_in(p_in, std::string("normal_x"));
    sensor_msgs::PointCloud2ConstIterator<float> y_n_in(p_in, std::string("normal_y"));
    sensor_msgs::PointCloud2ConstIterator<float> z_n_in(p_in, std::string("normal_z"));
    sensor_msgs::PointCloud2Iterator<float> x_n_out(p_out, std::string("normal_x"));
    sensor_msgs::PointCloud2Iterator<float> y_n_out(p_out, std::string("normal_y"));
    sensor_msgs::PointCloud2Iterator<float> z_n_out(p_out, std::string("normal_z"));

    for (; x_in != x_in.end();
      ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out,
      ++x_n_in, ++y_n_in, ++z_n_in, ++x_n_out, ++y_n_out, ++z_n_out)
    {
      point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);
      normal = r * Eigen::Vector3f(*x_n_in, *y_n_in, *z_n_in);
      *x_out = point.x();
      *y_out = point.y();
      *z_out = point.z();
      *x_n_out = normal.x();
      *y_n_out = normal.y();
      *z_n_out = normal.z();
    }
  }
}
inline
sensor_msgs::msg::PointCloud2 toMsg(const sensor_msgs::msg::PointCloud2 & in)
{
  return in;
}
inline
void fromMsg(const sensor_msgs::msg::PointCloud2 & msg, sensor_msgs::msg::PointCloud2 & out)
{
  out = msg;
}

}  // namespace tf2

#endif  // TF2_SENSOR_MSGS__TF2_SENSOR_MSGS_HPP_
