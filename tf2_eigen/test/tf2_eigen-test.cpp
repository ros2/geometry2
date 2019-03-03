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

/** \author Wim Meeussen */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

// To get M_PI, especially on Windows.
#include <math.h>

#include <tf2_eigen/tf2_eigen.h>
#include <gtest/gtest.h>
#include <tf2/convert.h>

// TODO(clalancette) Re-enable these tests once we have tf2/convert.h:convert(A, B) implemented
// TEST(TfEigen, ConvertVector3dStamped)
// {
//   const tf2::Stamped<Eigen::Vector3d> v(Eigen::Vector3d(1,2,3), tf2::TimePoint(std::chrono::seconds(5)), "test");

//   tf2::Stamped<Eigen::Vector3d> v1;
//   geometry_msgs::msg::PointStamped p1;
//   tf2::convert(v, p1);
//   tf2::convert(p1, v1);

//   EXPECT_EQ(v, v1);
// }

// TEST(TfEigen, ConvertVector3d)
// {
//   const Eigen::Vector3d v(1,2,3);

//   Eigen::Vector3d v1;
//   geometry_msgs::msg::Point p1;
//   tf2::convert(v, p1);
//   tf2::convert(p1, v1);

//   EXPECT_EQ(v, v1);
// }

// TEST(TfEigen, ConvertAffine3dStamped)
// {
//   const Eigen::Affine3d v_nonstamped(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));
//   const tf2::Stamped<Eigen::Affine3d> v(v_nonstamped, tf2::TimePoint(std::chrono::seconds(42)), "test_frame");

//   tf2::Stamped<Eigen::Affine3d> v1;
//   geometry_msgs::msg::PoseStamped p1;
//   tf2::convert(v, p1);
//   tf2::convert(p1, v1);

//   EXPECT_EQ(v.translation(), v1.translation());
//   EXPECT_EQ(v.rotation(), v1.rotation());
//   EXPECT_EQ(v.frame_id_, v1.frame_id_);
//   EXPECT_EQ(v.stamp_, v1.stamp_);
// }

// TEST(TfEigen, ConvertAffine3d)
// {
//   const Eigen::Affine3d v(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));

//   Eigen::Affine3d v1;
//   geometry_msgs::msg::Pose p1;
//   tf2::convert(v, p1);
//   tf2::convert(p1, v1);

//   EXPECT_EQ(v.translation(), v1.translation());
//   EXPECT_EQ(v.rotation(), v1.rotation());
// }

TEST(TfEigen, ConvertTransform)
{
  Eigen::Matrix4d tm;

  double alpha = M_PI/4.0;
  double theta = M_PI/6.0;
  double gamma = M_PI/12.0;

  tm << cos(theta)*cos(gamma),-cos(theta)*sin(gamma),sin(theta), 1, //
  cos(alpha)*sin(gamma)+sin(alpha)*sin(theta)*cos(gamma),cos(alpha)*cos(gamma)-sin(alpha)*sin(theta)*sin(gamma),-sin(alpha)*cos(theta), 2, //
  sin(alpha)*sin(gamma)-cos(alpha)*sin(theta)*cos(gamma),cos(alpha)*sin(theta)*sin(gamma)+sin(alpha)*cos(gamma),cos(alpha)*cos(theta), 3, //
  0, 0, 0, 1;

  Eigen::Affine3d T(tm);

  geometry_msgs::msg::TransformStamped msg = tf2::eigenToTransform(T);
  Eigen::Affine3d Tback = tf2::transformToEigen(msg);

  EXPECT_TRUE(T.isApprox(Tback));
  EXPECT_TRUE(tm.isApprox(Tback.matrix()));

  // same for Isometry
  Eigen::Isometry3d I(tm);

  msg = tf2::eigenToTransform(T);
  Eigen::Isometry3d Iback = tf2::transformToEigen(msg);

  EXPECT_TRUE(I.isApprox(Iback));
  EXPECT_TRUE(tm.isApprox(Iback.matrix()));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
