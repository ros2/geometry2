/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/** \author Wim Meeussen */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

// To get M_PI, especially on Windows.
#include <math.h>

#include <tf2_kdl/tf2_kdl.h>
#include <kdl/frames_io.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/buffer.h"
#include <tf2/convert.h>

std::unique_ptr<tf2_ros::Buffer> tf_buffer;
static const double EPS = 1e-3;

TEST(TfKDL, Frame)
{
  tf2::Stamped<KDL::Frame> v1(KDL::Frame(KDL::Rotation::RPY(M_PI, 0, 0), KDL::Vector(1,2,3)), tf2::TimePoint(tf2::durationFromSec(2.0)), "A");

  // simple api
  KDL::Frame v_simple = tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
  EXPECT_NEAR(v_simple.p[0], -9, EPS);
  EXPECT_NEAR(v_simple.p[1], 18, EPS);
  EXPECT_NEAR(v_simple.p[2], 27, EPS);
  double r, p, y;
  v_simple.M.GetRPY(r, p, y);
  EXPECT_NEAR(r, 0.0, EPS);
  EXPECT_NEAR(p, 0.0, EPS);
  EXPECT_NEAR(y, 0.0, EPS);


  // advanced api
  KDL::Frame v_advanced = tf_buffer->transform(v1, "B", tf2::TimePoint(tf2::durationFromSec(2.0)),
                "A", tf2::Duration(std::chrono::seconds(3)));
  EXPECT_NEAR(v_advanced.p[0], -9, EPS);
  EXPECT_NEAR(v_advanced.p[1], 18, EPS);
  EXPECT_NEAR(v_advanced.p[2], 27, EPS);
  v_advanced.M.GetRPY(r, p, y);
  EXPECT_NEAR(r, 0.0, EPS);
  EXPECT_NEAR(p, 0.0, EPS);
  EXPECT_NEAR(y, 0.0, EPS);

}

TEST(TfKDL, Vector)
{
  tf2::Stamped<KDL::Vector> v1(KDL::Vector(1,2,3),  tf2::TimePoint(tf2::durationFromSec(2.0)), "A");


  // simple api
  KDL::Vector v_simple = tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
  EXPECT_NEAR(v_simple[0], -9, EPS);
  EXPECT_NEAR(v_simple[1], 18, EPS);
  EXPECT_NEAR(v_simple[2], 27, EPS);

  // advanced api
  KDL::Vector v_advanced = tf_buffer->transform(v1, "B", tf2::TimePoint(tf2::durationFromSec(2.0)),
                "A", tf2::Duration(std::chrono::seconds(3)));
  EXPECT_NEAR(v_advanced[0], -9, EPS);
  EXPECT_NEAR(v_advanced[1], 18, EPS);
  EXPECT_NEAR(v_advanced[2], 27, EPS);
}

TEST(TfKDL, ConvertVector)
{
  tf2::Stamped<KDL::Vector> v(
      KDL::Vector(1,2,3),
      tf2::TimePoint(tf2::durationFromSec(1.0)),
      "my_frame"
  );
  tf2::Stamped<KDL::Vector> v1 = v;

  // Convert on duplicate arguments
  tf2::convert(v1, v1);
  EXPECT_EQ(v, v1);

  // Convert on same type
  tf2::Stamped<KDL::Vector> v2(
      KDL::Vector(3,4,5),
      tf2::TimePoint(tf2::durationFromSec(2.0)),
      "my_frame2"
  );
  tf2::convert(v1, v2);
  EXPECT_EQ(v, v2);
  EXPECT_EQ(v1, v2);

  // Round trip through corresponding message
  tf2::Stamped<KDL::Vector> v3;
  geometry_msgs::msg::PointStamped msg;
  tf2::convert(v, msg);
  tf2::convert(msg, v3);
  EXPECT_EQ(v, v3);
}

TEST(TfKDL, ConvertTwist)
{
  tf2::Stamped<KDL::Twist> t(
      KDL::Twist(KDL::Vector(1,2,3), KDL::Vector(4,5,6)),
      tf2::TimePoint(tf2::durationFromSec(1.0)),
      "my_frame");
  tf2::Stamped<KDL::Twist> t1 = t;

  // Test convert with duplicate arguments
  tf2::convert(t1, t1);
  EXPECT_EQ(t, t1);

  // Convert on same type
  tf2::Stamped<KDL::Twist> t2(
      KDL::Twist(KDL::Vector(3,4,5), KDL::Vector(6,7,8)),
      tf2::TimePoint(tf2::durationFromSec(2.0)),
      "my_frame2");
  tf2::convert(t1, t2);
  EXPECT_EQ(t, t2);
  EXPECT_EQ(t1, t2);

  // Round trip through corresponding message
  tf2::Stamped<KDL::Twist> t3;
  geometry_msgs::msg::TwistStamped msg;
  tf2::convert(t, msg);
  tf2::convert(msg, t3);
  EXPECT_EQ(t, t3);
}

TEST(TfKDL, ConvertWrench)
{
  tf2::Stamped<KDL::Wrench> w(
      KDL::Wrench(KDL::Vector(1,2,3), KDL::Vector(4,5,6)),
      tf2::TimePoint(tf2::durationFromSec(1.0)),
      "my_frame");
  tf2::Stamped<KDL::Wrench> w1 = w;

  // Test convert with duplicate arguments
  tf2::convert(w1, w1);
  EXPECT_EQ(w, w1);

  // Convert on same type
  tf2::Stamped<KDL::Wrench> w2(
      KDL::Wrench(KDL::Vector(3,4,5), KDL::Vector(6,7,8)),
      tf2::TimePoint(tf2::durationFromSec(2.0)),
      "my_frame2");
  tf2::convert(w1, w2);
  EXPECT_EQ(w, w2);
  EXPECT_EQ(w1, w2);

  // Round trip through corresponding message
  tf2::Stamped<KDL::Wrench> w3;
  geometry_msgs::msg::WrenchStamped msg;
  tf2::convert(w, msg);
  tf2::convert(msg, w3);
  EXPECT_EQ(w, w3);
}

TEST(TfKDL, ConvertFrame)
{
  tf2::Stamped<KDL::Frame> f(
      KDL::Frame(KDL::Rotation::RPY(M_PI, 0, 0), KDL::Vector(1,2,3)),
      tf2::TimePoint(tf2::durationFromSec(1.0)),
      "my_frame");
  tf2::Stamped<KDL::Frame> f1 = f;

  // Test convert with duplicate arguments
  tf2::convert(f1, f1);
  EXPECT_EQ(f, f1);

  // Convert on same type
  tf2::Stamped<KDL::Frame> f2(
      KDL::Frame(KDL::Rotation::RPY(0, M_PI, 0), KDL::Vector(4,5,6)),
      tf2::TimePoint(tf2::durationFromSec(2.0)),
      "my_frame2");
  tf2::convert(f1, f2);
  EXPECT_EQ(f, f2);
  EXPECT_EQ(f1, f2);

  // Round trip through corresponding message
  tf2::Stamped<KDL::Frame> f3;
  geometry_msgs::msg::PoseStamped msg;
  tf2::convert(f, msg);
  tf2::convert(msg, f3);
  EXPECT_EQ(f, f3);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);

  // populate buffer
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.w = 0;
  t.transform.rotation.x = 1;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.header.stamp.sec = 2;
  t.header.stamp.nanosec = 0;
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  tf_buffer->setTransform(t, "test");

  int retval = RUN_ALL_TESTS();
  return retval;
}
