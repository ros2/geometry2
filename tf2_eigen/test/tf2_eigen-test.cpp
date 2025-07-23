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

// To get M_PI, especially on Windows.

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <cmath>
#include <memory>

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
#include <Eigen/Geometry>  // NOLINT
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include "gtest/gtest.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"

#include "tf2/convert.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

TEST(TfEigen, ConvertVector3dStamped)
{
  const tf2::Stamped<Eigen::Vector3d> v(Eigen::Vector3d(1, 2, 3), tf2::TimePoint(
      std::chrono::seconds(5)), "test");

  tf2::Stamped<Eigen::Vector3d> v1;
  geometry_msgs::msg::PointStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v, v1);
}

// TODO(clalancette) Re-enable these tests once we have tf2/convert.h:convert(A, B) implemented
// TEST(TfEigen, ConvertVector3d)
// {
//   const Eigen::Vector3d v(1,2,3);

//   Eigen::Vector3d v1;
//   geometry_msgs::msg::Point p1;
//   tf2::convert(v, p1);
//   tf2::convert(p1, v1);

//   EXPECT_EQ(v, v1);
// }

TEST(TfEigen, ConvertAffine3dStamped)
{
  const Eigen::Affine3d v_nonstamped(Eigen::Translation3d(1, 2, 3) * Eigen::AngleAxis<double>(
      1, Eigen::Vector3d::UnitX()));
  const tf2::Stamped<Eigen::Affine3d> v(v_nonstamped, tf2::TimePoint(
      std::chrono::seconds(42)), "test_frame");

  tf2::Stamped<Eigen::Affine3d> v1;
  geometry_msgs::msg::PoseStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
}

// TODO(clalancette) Re-enable these tests once we have tf2/convert.h:convert(A, B) implemented
// TEST(TfEigen, ConvertAffine3d)
// {
//   const Eigen::Affine3d v(
//     Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));

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

  double alpha = M_PI / 4.0;
  double theta = M_PI / 6.0;
  double gamma = M_PI / 12.0;

  tm << cos(theta) * cos(gamma), -cos(theta) * sin(gamma), sin(theta), 1,
    cos(alpha) * sin(gamma) + sin(alpha) * sin(theta) * cos(gamma),
    cos(alpha) * cos(gamma) - sin(alpha) * sin(theta) * sin(gamma), -sin(alpha) * cos(theta), 2,
    sin(alpha) * sin(gamma) - cos(alpha) * sin(theta) * cos(gamma),
    cos(alpha) * sin(theta) * sin(gamma) + sin(alpha) * cos(gamma), cos(alpha) * cos(theta), 3,
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

struct EigenBufferTransform : public ::testing::Test
{
  static void SetUpTestSuite()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
    tf_buffer->setUsingDedicatedThread(true);

    // populate buffer
    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = 10;
    t.transform.translation.y = 20;
    t.transform.translation.z = 30;
    t.transform.rotation.w = 0;
    t.transform.rotation.x = 1;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    t.header.frame_id = "A";
    t.child_frame_id = "B";
    tf_buffer->setTransform(t, "test");
  }

  template<int mode>
  void testEigenTransform();

  ::testing::AssertionResult doTestEigenQuaternion(
    const Eigen::Quaterniond & parameter, const Eigen::Quaterniond & expected);

  static std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  static constexpr double EPS = 1e-3;
};

std::unique_ptr<tf2_ros::Buffer> EigenBufferTransform::tf_buffer;

template<int mode>
void EigenBufferTransform::testEigenTransform()
{
  using T = Eigen::Transform<double, 3, mode>;
  using stampedT = tf2::Stamped<T>;

  const stampedT i1{
    T{Eigen::Translation3d{1, 2, 3} *Eigen::Quaterniond{0, 1, 0, 0}}, tf2::timeFromSec(2), "A"};

  // simple api
  const stampedT i_simple = tf_buffer->transform(i1, "B", tf2::durationFromSec(2.0));
  EXPECT_NEAR(i_simple.translation().x(), -9, EPS);
  EXPECT_NEAR(i_simple.translation().y(), 18, EPS);
  EXPECT_NEAR(i_simple.translation().z(), 27, EPS);
  const auto q1 = Eigen::Quaterniond(i_simple.linear());
  EXPECT_NEAR(q1.x(), 0.0, EPS);
  EXPECT_NEAR(q1.y(), 0.0, EPS);
  EXPECT_NEAR(q1.z(), 0.0, EPS);
  EXPECT_NEAR(q1.w(), 1.0, EPS);

  // advanced api
  const stampedT v_advanced =
    tf_buffer->transform(i1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
  EXPECT_NEAR(v_advanced.translation().x(), -9, EPS);
  EXPECT_NEAR(v_advanced.translation().y(), 18, EPS);
  EXPECT_NEAR(v_advanced.translation().z(), 27, EPS);
  const auto q2 = Eigen::Quaterniond(v_advanced.linear());
  EXPECT_NEAR(q2.x(), 0.0, EPS);
  EXPECT_NEAR(q2.y(), 0.0, EPS);
  EXPECT_NEAR(q2.z(), 0.0, EPS);
  EXPECT_NEAR(q2.w(), 1.0, EPS);
}

TEST_F(EigenBufferTransform, Affine3d) {
  testEigenTransform<Eigen::Affine>();
}

TEST_F(EigenBufferTransform, Isometry3d) {
  testEigenTransform<Eigen::Isometry>();
}

TEST_F(EigenBufferTransform, Vector)
{
  const tf2::Stamped<Eigen::Vector3d> v1{{1, 2, 3}, tf2::timeFromSec(2), "A"};

  // simple api
  const tf2::Stamped<Eigen::Vector3d> v_simple =
    tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
  EXPECT_NEAR(v_simple.x(), -9, EPS);
  EXPECT_NEAR(v_simple.y(), 18, EPS);
  EXPECT_NEAR(v_simple.z(), 27, EPS);

  // advanced api
  const tf2::Stamped<Eigen::Vector3d> v_advanced =
    tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
  EXPECT_NEAR(v_advanced.x(), -9, EPS);
  EXPECT_NEAR(v_advanced.y(), 18, EPS);
  EXPECT_NEAR(v_advanced.z(), 27, EPS);
}

// Test transformation of a 6-long wrench or twist
TEST_F(EigenBufferTransform, WrenchTransform)
{
  // Transform the wrench (due to gravity) of a point mass to a different frame

  double mass = 1.0;
  double gravity = -9.81;
  // Negative y force, no moment
  Eigen::VectorXd wrench_in(6);
  wrench_in << 0., mass * gravity, 0., 0., 0., 0.;

  // The new frame is not rotated at all but it is offset along x-axis
  double x_offset = -0.1;
  geometry_msgs::msg::TransformStamped tf_to_new_frame;
  tf_to_new_frame.transform.translation.x = x_offset;
  tf_to_new_frame.transform.rotation.w = 1.0;

  Eigen::VectorXd wrench_out(6);
  tf2::doTransform(wrench_in, wrench_out, tf_to_new_frame);

  EXPECT_NEAR(wrench_out(0), 0., EPS);
  EXPECT_NEAR(wrench_out(1), mass * gravity, EPS);
  EXPECT_NEAR(wrench_out(2), 0., EPS);
  EXPECT_NEAR(wrench_out(3), 0., EPS);
  EXPECT_NEAR(wrench_out(4), 0., EPS);
  EXPECT_NEAR(wrench_out(5), mass * gravity * x_offset, EPS);
}

// helper method for Quaternion tests
::testing::AssertionResult EigenBufferTransform::doTestEigenQuaternion(
  const Eigen::Quaterniond & parameter, const Eigen::Quaterniond & expected)
{
  const tf2::Stamped<Eigen::Quaterniond> q1{parameter, tf2::timeFromSec(2), "A"};
  // avoid linking error
  const double eps = EPS;

  // simple api
  const tf2::Stamped<Eigen::Quaterniond> q_simple =
    tf_buffer->transform(q1, "B", tf2::durationFromSec(2.0));
  // compare rotation matrices, as the quaternions can be ambigous
  EXPECT_TRUE(q_simple.toRotationMatrix().isApprox(expected.toRotationMatrix(), eps));

  // advanced api
  const tf2::Stamped<Eigen::Quaterniond> q_advanced =
    tf_buffer->transform(q1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
  EXPECT_TRUE(q_advanced.toRotationMatrix().isApprox(expected.toRotationMatrix(), eps));
  return ::testing::AssertionSuccess();
}

TEST_F(EigenBufferTransform, QuaternionRotY)
{
  // rotated by -90° around y
  // 0, 0, -1
  // 0, 1, 0,
  // 1, 0, 0
  const Eigen::Quaterniond param{Eigen::AngleAxisd(-1 * M_PI_2, Eigen::Vector3d::UnitY())};
  const Eigen::Quaterniond expected{0, M_SQRT1_2, 0, -1 * M_SQRT1_2};
  EXPECT_TRUE(doTestEigenQuaternion(param, expected));
}

TEST_F(EigenBufferTransform, QuaternionRotX)
{
  // rotated by 90° around y
  const Eigen::Quaterniond param{Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX())};
  const Eigen::Quaterniond expected{Eigen::AngleAxisd(-1 * M_PI_2, Eigen::Vector3d::UnitX())};
  EXPECT_TRUE(doTestEigenQuaternion(param, expected));
}

TEST_F(EigenBufferTransform, QuaternionRotZ)
{
  // rotated by 180° around z
  const Eigen::Quaterniond param{Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())};
  const Eigen::Quaterniond expected{Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())};
  EXPECT_TRUE(doTestEigenQuaternion(param, expected));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
