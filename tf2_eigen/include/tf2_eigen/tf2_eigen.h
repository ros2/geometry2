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

/** \author Koji Terada */

#ifndef TF2_EIGEN_H
#define TF2_EIGEN_H

#include <chrono>

#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


namespace tf2
{

Eigen::Affine3d transformToEigen(const geometry_msgs::msg::TransformStamped& t) {
  return Eigen::Affine3d(Eigen::Translation3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
			 * Eigen::Quaterniond(t.transform.rotation.w,
					      t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z));
}


// this method needs to be implemented by client library developers
template <>
void doTransform(const tf2::Stamped<Eigen::Vector3d>& t_in,
		 tf2::Stamped<Eigen::Vector3d>& t_out,
		 const geometry_msgs::msg::TransformStamped& transform) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted{std::chrono::nanoseconds{transform.header.stamp.sec*1000000000LL+transform.header.stamp.nanosec}};
  std::chrono::system_clock::time_point recovered = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted);
  t_out = tf2::Stamped<Eigen::Vector3d>(transformToEigen(transform) * t_in,
                                        recovered,
                                        transform.header.frame_id);
}

//convert to vector message
geometry_msgs::msg::PointStamped toMsg(const tf2::Stamped<Eigen::Vector3d>& in)
{
  geometry_msgs::msg::PointStamped msg;
  auto nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(in.stamp_.time_since_epoch());
  msg.header.stamp.sec = nanosecs.count() / 1000000000LL;
  msg.header.stamp.nanosec = (nanosecs.count() % 1000000000LL) * 1000000000LL;
  msg.header.frame_id = in.frame_id_;
  msg.point.x = in.x();
  msg.point.y = in.y();
  msg.point.z = in.z();
  return msg;
}

void fromMsg(const geometry_msgs::msg::PointStamped& msg, tf2::Stamped<Eigen::Vector3d>& out) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted{std::chrono::nanoseconds{msg.header.stamp.sec*1000000000LL+msg.header.stamp.nanosec}};
  std::chrono::system_clock::time_point recovered = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted);

  out.stamp_ = recovered;
  out.frame_id_ = msg.header.frame_id;
  out.x() = msg.point.x;
  out.y() = msg.point.y;
  out.z() = msg.point.z;
}


// this method needs to be implemented by client library developers
template <>
void doTransform(const tf2::Stamped<Eigen::Affine3d>& t_in,
		 tf2::Stamped<Eigen::Affine3d>& t_out,
		 const geometry_msgs::msg::TransformStamped& transform) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted{std::chrono::nanoseconds{transform.header.stamp.sec*1000000000LL+transform.header.stamp.nanosec}};
  std::chrono::system_clock::time_point recovered = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted);

  t_out = tf2::Stamped<Eigen::Affine3d>(transformToEigen(transform) * t_in, recovered, transform.header.frame_id);
}

//convert to pose message
geometry_msgs::msg::PoseStamped toMsg(const tf2::Stamped<Eigen::Affine3d>& in)
{
  geometry_msgs::msg::PoseStamped msg;
  auto nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(in.stamp_.time_since_epoch());
  msg.header.stamp.sec = nanosecs.count() / 1000000000LL;
  msg.header.stamp.nanosec = (nanosecs.count() % 1000000000LL) * 1000000000LL;
  msg.header.frame_id = in.frame_id_;
  msg.pose.position.x = in.translation().x();
  msg.pose.position.y = in.translation().y();
  msg.pose.position.z = in.translation().z();
  msg.pose.orientation.x = Eigen::Quaterniond(in.rotation()).x();
  msg.pose.orientation.y = Eigen::Quaterniond(in.rotation()).y();
  msg.pose.orientation.z = Eigen::Quaterniond(in.rotation()).z();
  msg.pose.orientation.w = Eigen::Quaterniond(in.rotation()).w();
  return msg;
}

void fromMsg(const geometry_msgs::msg::PoseStamped& msg, tf2::Stamped<Eigen::Affine3d>& out)
{
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted{std::chrono::nanoseconds{msg.header.stamp.sec*1000000000LL+msg.header.stamp.nanosec}};
  std::chrono::system_clock::time_point recovered = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted);
  out.stamp_ = recovered;
  out.frame_id_ = msg.header.frame_id;
  out.setData(Eigen::Affine3d(Eigen::Translation3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                              * Eigen::Quaterniond(msg.pose.orientation.w,
                                                   msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)));
}

} // namespace

#endif // TF2_EIGEN_H
